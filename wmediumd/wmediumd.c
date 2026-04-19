/*
 *	wmediumd, wireless medium simulator for mac80211_hwsim kernel module
 *	Copyright (c) 2011 cozybit Inc.
 *
 *	Author:	Javier Lopez	<jlopex@cozybit.com>
 *		Javier Cardona	<javier@cozybit.com>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version 2
 *	of the License, or (at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 *	02110-1301, USA.
 */

#include <netlink/netlink.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/ctrl.h>
#include <netlink/genl/family.h>
#include <stdint.h>
#include <getopt.h>
#include <signal.h>
#include <event.h>
#include <math.h>
#include <sys/timerfd.h>
#include <errno.h>
#include <limits.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>

#include "wmediumd.h"
#include "ieee80211.h"
#include "config.h"
#include "wserver.h"
#include "wmediumd_dynamic.h"
#include "wserver_messages.h"

struct sockaddr_in serverAddr, clientAddr;
socklen_t len;
void* out_buf;
char in_buf[PAGE_SIZE];
static bool is_ap = true;

static inline int div_round(int a, int b)
{
	return (a + b - 1) / b;
}

static inline int pkt_duration(struct wmediumd *ctx, int len, int rate)
{
	/* preamble + signal + t_sym * n_sym, rate in 100 kbps */
	return 16 + 4 + 4 * div_round((16 + 8 * len + 6) * 10, 4 * rate);
}

int w_logf(struct wmediumd *ctx, u8 level, const char *format, ...)
{
	va_list(args);
	va_start(args, format);
	if (ctx->log_lvl >= level) {
		return vprintf(format, args);
	}
	return -1;
}

int w_flogf(struct wmediumd *ctx, u8 level, FILE *stream, const char *format, ...)
{
	va_list(args);
	va_start(args, format);
	if (ctx->log_lvl >= level) {
		return vfprintf(stream, format, args);
	}
	return -1;
}

static void wqueue_init(struct wqueue *wqueue, int cw_min, int cw_max)
{
	INIT_LIST_HEAD(&wqueue->frames);
	wqueue->cw_min = cw_min;
	wqueue->cw_max = cw_max;
}

static void rx_window_init(struct rx_window *rx_window)
{
	size_t i;

	rx_window->num_slots = RX_WINDOW_SLOTS;
	rx_window->slots = calloc(rx_window->num_slots, sizeof(*rx_window->slots));
	if (!rx_window->slots) {
		rx_window->num_slots = 0;
		return;
	}

	for (i = 0; i < rx_window->num_slots; i++)
		rx_window->slots[i].slot_idx = ULLONG_MAX;
}

void station_init_queues(struct station *station)
{
	wqueue_init(&station->queues[IEEE80211_AC_BK], 15, 1023);
	wqueue_init(&station->queues[IEEE80211_AC_BE], 15, 1023);
	wqueue_init(&station->queues[IEEE80211_AC_VI], 7, 15);
	wqueue_init(&station->queues[IEEE80211_AC_VO], 3, 7);
	rx_window_init(&station->rx_window);
}

void station_free_resources(struct station *station)
{
	free(station->rx_window.slots);
	station->rx_window.slots = NULL;
	station->rx_window.num_slots = 0;
}

bool timespec_before(struct timespec *t1, struct timespec *t2)
{
	return t1->tv_sec < t2->tv_sec ||
	       (t1->tv_sec == t2->tv_sec && t1->tv_nsec < t2->tv_nsec);
}

void timespec_add_usec(struct timespec *t, int usec)
{
	t->tv_nsec += usec * 1000;
	if (t->tv_nsec >= 1000000000) {
		t->tv_sec++;
		t->tv_nsec -= 1000000000;
	}
}

// a - b = c
static int timespec_sub(struct timespec *a, struct timespec *b,
			struct timespec *c)
{
	c->tv_sec = a->tv_sec - b->tv_sec;

	if (a->tv_nsec < b->tv_nsec) {
		c->tv_sec--;
		c->tv_nsec = 1000000000 + a->tv_nsec - b->tv_nsec;
	} else {
		c->tv_nsec = a->tv_nsec - b->tv_nsec;
	}

	return 0;
}

static u64 timespec_to_usec(const struct timespec *t)
{
	return (u64)t->tv_sec * 1000000ULL + (u64)t->tv_nsec / 1000ULL;
}

static void timespec_add_usec64(struct timespec *t, u64 usec)
{
	t->tv_sec += usec / 1000000ULL;
	t->tv_nsec += (long)((usec % 1000000ULL) * 1000ULL);
	if (t->tv_nsec >= 1000000000L) {
		t->tv_sec += t->tv_nsec / 1000000000L;
		t->tv_nsec %= 1000000000L;
	}
}

static struct timespec timespec_plus_usec(const struct timespec *base, u64 usec)
{
	struct timespec out = *base;

	timespec_add_usec64(&out, usec);
	return out;
}

static struct timespec timespec_from_usec(u64 usec)
{
	struct timespec out;

	out.tv_sec = usec / 1000000ULL;
	out.tv_nsec = (long)((usec % 1000000ULL) * 1000ULL);
	return out;
}

static u64 interpolate_u64(int value, int start, int end,
			   u64 start_val, u64 end_val)
{
	u64 span;
	u64 delta;

	if (start >= end || value <= start)
		return start_val;
	if (value >= end)
		return end_val;

	span = (u64)(end - start);
	delta = (u64)(value - start);
	if (end_val >= start_val)
		return start_val + ((end_val - start_val) * delta) / span;

	return start_val - ((start_val - end_val) * delta) / span;
}

static unsigned int interpolate_pct(int value, int start, int end,
				    unsigned int start_pct,
				    unsigned int end_pct)
{
	return (unsigned int)interpolate_u64(value, start, end,
					     start_pct, end_pct);
}

static struct timespec timespec_forever(void)
{
	struct timespec out;

	out.tv_sec = LONG_MAX / 4;
	out.tv_nsec = 0;
	return out;
}

static inline bool rx_window_enabled(struct wmediumd *ctx)
{
	return ctx->intf != NULL;
}

static inline bool same_medium(struct station *a, struct station *b)
{
	return a->medium_id == b->medium_id;
}

struct medium_interval {
	struct timespec start;
	struct timespec end;
};

static bool is_multicast_ether_addr(const u8 *addr);
static struct station *get_station_by_addr(struct wmediumd *ctx, u8 *addr);
static void schedule_frame_attempt(struct wmediumd *ctx, struct station *station,
				   struct frame *frame,
				   const struct timespec *base_time);

static void clear_frame_schedule(struct frame *frame)
{
	struct timespec forever = timespec_forever();

	frame->attempt_scheduled = false;
	frame->ppdu_start = forever;
	frame->ppdu_end = forever;
	frame->expires = forever;
}

static struct frame *queue_head_frame(struct wqueue *queue)
{
	return list_first_entry_or_null(&queue->frames, struct frame, list);
}

static struct timespec recent_busy_sync_base(struct wmediumd *ctx,
					     const struct timespec *now)
{
	struct timespec now_copy = *now;
	u64 now_us;
	u64 busy_end_us;
	u64 gap_us;
	u64 sync_grace_us;
	u64 adjusted_gap_us;
	unsigned int pull_pct;

	if (ctx->enable_medium_detection)
		return *now;
	if (!ctx->last_global_busy_end.tv_sec && !ctx->last_global_busy_end.tv_nsec)
		return *now;
	if (timespec_before(&now_copy, &ctx->last_global_busy_end))
		return ctx->last_global_busy_end;

	now_us = timespec_to_usec(now);
	busy_end_us = timespec_to_usec(&ctx->last_global_busy_end);
	gap_us = now_us - busy_end_us;

	if (ctx->num_stas <= QUEUE_SYNC_RELAXED_STAS) {
		sync_grace_us = interpolate_u64(ctx->num_stas,
						CONTENTION_CURVE_MIN_STAS,
						QUEUE_SYNC_RELAXED_STAS,
						QUEUE_SYNC_GRACE_US,
						RELAXED_NODE_QUEUE_SYNC_GRACE_US);
		pull_pct = interpolate_pct(ctx->num_stas,
					   CONTENTION_CURVE_MIN_STAS,
					   QUEUE_SYNC_RELAXED_STAS,
					   QUEUE_SYNC_PULL_BASE_PCT,
					   QUEUE_SYNC_PULL_RELAXED_PCT);
	} else if (ctx->num_stas <= QUEUE_SYNC_FULL_STAS) {
		sync_grace_us = interpolate_u64(ctx->num_stas,
						QUEUE_SYNC_RELAXED_STAS,
						QUEUE_SYNC_FULL_STAS,
						RELAXED_NODE_QUEUE_SYNC_GRACE_US,
						LOW_NODE_QUEUE_SYNC_GRACE_US);
		pull_pct = interpolate_pct(ctx->num_stas,
					   QUEUE_SYNC_RELAXED_STAS,
					   QUEUE_SYNC_FULL_STAS,
					   QUEUE_SYNC_PULL_RELAXED_PCT,
					   QUEUE_SYNC_PULL_FULL_PCT);
	} else if (ctx->num_stas <= MID_NODE_RX_WINDOW_MAX_STAS) {
		sync_grace_us = LOW_NODE_QUEUE_SYNC_GRACE_US;
		pull_pct = QUEUE_SYNC_PULL_FULL_PCT;
	} else if (ctx->num_stas <= QUEUE_SYNC_TAIL_STAS) {
		sync_grace_us = interpolate_u64(ctx->num_stas,
						MID_NODE_RX_WINDOW_MAX_STAS,
						QUEUE_SYNC_TAIL_STAS,
						LOW_NODE_QUEUE_SYNC_GRACE_US,
						HIGH_NODE_QUEUE_SYNC_GRACE_US);
		pull_pct = interpolate_pct(ctx->num_stas,
					   MID_NODE_RX_WINDOW_MAX_STAS,
					   QUEUE_SYNC_TAIL_STAS,
					   QUEUE_SYNC_PULL_FULL_PCT,
					   QUEUE_SYNC_PULL_HIGH_PCT);
	} else {
		sync_grace_us = HIGH_NODE_QUEUE_SYNC_GRACE_US;
		pull_pct = QUEUE_SYNC_PULL_HIGH_PCT;
	}

	if (gap_us > sync_grace_us || !pull_pct)
		return *now;

	/*
	 * 不再用“命中阈值就完全回拉”的硬切换，而是让回拉强度随着
	 * 空窗长度逐步衰减。短空窗更接近上一轮 busy 结束时刻，
	 * 较长空窗则逐渐回到本地入队时间。
	 */
	pull_pct = (unsigned int)(((u64)pull_pct * (sync_grace_us - gap_us)) /
				  sync_grace_us);
	adjusted_gap_us = (gap_us * (100U - pull_pct)) / 100U;
	return timespec_plus_usec(&ctx->last_global_busy_end, adjusted_gap_us);
}

static void schedule_queue_head_frame(struct wmediumd *ctx,
				      struct station *station, int ac,
				      const struct timespec *base_time)
{
	struct frame *frame = queue_head_frame(&station->queues[ac]);
	struct timespec target = *base_time;
	int i;

	if (!frame || frame->attempt_scheduled)
		return;

	for (i = 0; i <= ac; i++) {
		struct frame *head = queue_head_frame(&station->queues[i]);

		if (!head || head == frame || !head->attempt_scheduled)
			continue;
		if (timespec_before(&target, &head->expires))
			target = head->expires;
	}

	schedule_frame_attempt(ctx, station, frame, &target);
}

void rearm_timer(struct wmediumd *ctx)
{
	struct timespec min_expires;
	struct itimerspec expires;
	struct station *station;
	struct frame *frame;
	int i;

	bool set_min_expires = false;

	/*
	 * Iterate over all the interfaces to find the next frame that
	 * will be delivered, and set the timerfd accordingly.
	 */
	list_for_each_entry(station, &ctx->stations, list) {
		for (i = 0; i < IEEE80211_NUM_ACS; i++) {
			frame = list_first_entry_or_null(&station->queues[i].frames,
							 struct frame, list);

			if (frame && (!set_min_expires ||
				      timespec_before(&frame->expires,
						      &min_expires))) {
				set_min_expires = true;
				min_expires = frame->expires;
			}
		}
	}

	if (set_min_expires) {
		memset(&expires, 0, sizeof(expires));
		expires.it_value = min_expires;
		timerfd_settime(ctx->timerfd, TFD_TIMER_ABSTIME, &expires,
				NULL);
	}
}

static inline bool frame_has_a4(struct frame *frame)
{
	struct ieee80211_hdr *hdr = (void *)frame->data;

	return (hdr->frame_control[1] & (FCTL_TODS | FCTL_FROMDS)) ==
		(FCTL_TODS | FCTL_FROMDS);
}

static inline bool frame_is_mgmt(struct frame *frame)
{
	struct ieee80211_hdr *hdr = (void *)frame->data;

	return (hdr->frame_control[0] & FCTL_FTYPE) == FTYPE_MGMT;
}

static inline bool frame_is_data(struct frame *frame)
{
	struct ieee80211_hdr *hdr = (void *)frame->data;

	return (hdr->frame_control[0] & FCTL_FTYPE) == FTYPE_DATA;
}

static inline bool frame_is_data_qos(struct frame *frame)
{
	struct ieee80211_hdr *hdr = (void *)frame->data;

	return (hdr->frame_control[0] & (FCTL_FTYPE | STYPE_QOS_DATA)) ==
		(FTYPE_DATA | STYPE_QOS_DATA);
}

static inline u8 *frame_get_qos_ctl(struct frame *frame)
{
	struct ieee80211_hdr *hdr = (void *)frame->data;

	if (frame_has_a4(frame))
		return (u8 *)hdr + 30;
	else
		return (u8 *)hdr + 24;
}

static enum ieee80211_ac_number frame_select_queue_80211(struct frame *frame)
{
	u8 *p;
	int priority;

	if (!frame_is_data(frame))
		return IEEE80211_AC_VO;

	if (!frame_is_data_qos(frame))
		return IEEE80211_AC_BE;

	p = frame_get_qos_ctl(frame);
	priority = *p & QOS_CTL_TAG1D_MASK;

	return ieee802_1d_to_ac[priority];
}

static double dBm_to_milliwatt(int decibel_intf)
{
#define INTF_LIMIT (31)
	int intf_diff = NOISE_LEVEL - decibel_intf;

	if (intf_diff >= INTF_LIMIT)
		return 0.001;

	if (intf_diff <= -INTF_LIMIT)
		return 1000.0;

	return pow(10.0, -intf_diff / 10.0);
}

static double milliwatt_to_dBm(double value)
{
	return 10.0 * log10(value);
}

static int get_signal_dbm(struct wmediumd *ctx, struct station *src,
			  struct station *dst, bool include_fading)
{
	int snr;

	snr = ctx->get_link_snr(ctx, src, dst);
	if (include_fading)
		snr += ctx->get_fading_signal(ctx);

	return snr + NOISE_LEVEL;
}

static void rx_window_add_power(struct rx_window *rx_window, u64 slot_idx,
				double power_mw)
{
	struct rx_slot *slot;

	if (!rx_window->slots || !rx_window->num_slots)
		return;

	slot = &rx_window->slots[slot_idx % rx_window->num_slots];
	if (slot->slot_idx != slot_idx) {
		slot->slot_idx = slot_idx;
		slot->power_mw = 0.0;
	}

	slot->power_mw += power_mw;
	if (slot->power_mw < 0.0)
		slot->power_mw = 0.0;
}

static double rx_window_get_power(struct rx_window *rx_window, u64 slot_idx)
{
	struct rx_slot *slot;

	if (!rx_window->slots || !rx_window->num_slots)
		return 0.0;

	slot = &rx_window->slots[slot_idx % rx_window->num_slots];
	if (slot->slot_idx != slot_idx)
		return 0.0;

	return slot->power_mw;
}

static u64 rx_window_guard_us(struct wmediumd *ctx)
{
	if (ctx->num_stas <= LOW_NODE_RX_WINDOW_MAX_STAS)
		return RX_WINDOW_GUARD_LOW_US;
	if (ctx->num_stas <= MID_NODE_RX_WINDOW_MAX_STAS)
		return RX_WINDOW_GUARD_MID_US;
	if (ctx->num_stas <= RX_WINDOW_GUARD_TAIL_STAS)
		return interpolate_u64(ctx->num_stas,
				       MID_NODE_RX_WINDOW_MAX_STAS,
				       RX_WINDOW_GUARD_TAIL_STAS,
				       RX_WINDOW_GUARD_MID_US,
				       RX_WINDOW_GUARD_HIGH_US);

	return RX_WINDOW_GUARD_HIGH_US;
}

static void reserve_ppdu_for_receivers_delta(struct wmediumd *ctx,
					     struct station *sender,
					     const struct timespec *ppdu_start,
					     const struct timespec *ppdu_end,
					     double delta)
{
	struct station *receiver;
	double signal_mw;
	u64 start_slot, end_slot, slot_idx;
	u64 start_us, end_us, guard_us;

	if (!rx_window_enabled(ctx))
		return;

	start_us = timespec_to_usec(ppdu_start);
	end_us = timespec_to_usec(ppdu_end);
	guard_us = rx_window_guard_us(ctx);

	if (start_us > guard_us)
		start_us -= guard_us;
	else
		start_us = 0;
	end_us += guard_us;

	start_slot = start_us / RX_WINDOW_SLOT_US;
	end_slot = div_round((int)(end_us - start_us), RX_WINDOW_SLOT_US);
	if (!end_slot)
		end_slot = 1;

	list_for_each_entry(receiver, &ctx->stations, list) {
		int signal_dbm;

		if (receiver == sender || !same_medium(sender, receiver))
			continue;

		signal_dbm = get_signal_dbm(ctx, sender, receiver, false);
		signal_mw = dBm_to_milliwatt(signal_dbm) * delta;

		for (slot_idx = 0; slot_idx < end_slot; slot_idx++)
			rx_window_add_power(&receiver->rx_window,
					    start_slot + slot_idx, signal_mw);
	}
}

static void reserve_ppdu_for_receivers(struct wmediumd *ctx, struct station *sender,
				       const struct timespec *ppdu_start,
				       const struct timespec *ppdu_end)
{
	reserve_ppdu_for_receivers_delta(ctx, sender, ppdu_start, ppdu_end, 1.0);
}

static void release_ppdu_for_receivers(struct wmediumd *ctx, struct station *sender,
				       const struct timespec *ppdu_start,
				       const struct timespec *ppdu_end)
{
	reserve_ppdu_for_receivers_delta(ctx, sender, ppdu_start, ppdu_end, -1.0);
}

static void configure_best_effort_socket(int sock)
{
	int enable = 1;
	int buf_size = 1 << 20;

	setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
	setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size));
	setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &buf_size, sizeof(buf_size));
}

static ssize_t sendto_retry(int sock, const void *buf, size_t len, int flags,
			    const struct sockaddr *dest_addr,
			    socklen_t addrlen)
{
	ssize_t ret;

	do {
		ret = sendto(sock, buf, len, flags, dest_addr, addrlen);
	} while (ret < 0 && errno == EINTR);

	return ret;
}

static ssize_t recvfrom_retry(int sock, void *buf, size_t len, int flags,
			      struct sockaddr *src_addr, socklen_t *addrlen)
{
	ssize_t ret;

	do {
		ret = recvfrom(sock, buf, len, flags, src_addr, addrlen);
	} while (ret < 0 && errno == EINTR);

	if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
		return 0;

	return ret;
}

/*
 * 依据接收端 4us 窗口中的累计功率，逐符号估算等效 BER。
 * exclude_self=true 时，会从窗口累计值中扣除当前目标帧自身功率。
 */
static double calc_avg_ber_from_window(struct station *receiver,
				       const struct timespec *ppdu_start,
				       const struct timespec *ppdu_end,
				       double signal_mw, bool exclude_self,
				       unsigned int rate_idx, u32 freq)
{
	u64 start_slot, slot_count, slot_idx;
	double ber_sum = 0.0;

	if (signal_mw <= 1.0)
		return 1.0;

	start_slot = timespec_to_usec(ppdu_start) / RX_WINDOW_SLOT_US;
	slot_count = div_round((int)(timespec_to_usec(ppdu_end) -
				 timespec_to_usec(ppdu_start)),
			      RX_WINDOW_SLOT_US);
	if (!slot_count)
		slot_count = 1;

	for (slot_idx = 0; slot_idx < slot_count; slot_idx++) {
		double total_power;
		double intf_power;
		double sinr_linear;
		double sinr_db;

		total_power = rx_window_get_power(&receiver->rx_window,
						  start_slot + slot_idx);
		if (exclude_self) {
			total_power -= signal_mw;
			if (total_power < 0.0)
				total_power = 0.0;
		}

		intf_power = 1.0 + total_power;
		sinr_linear = signal_mw / intf_power;
		sinr_db = milliwatt_to_dBm(sinr_linear);
		ber_sum += get_bit_error_prob_from_snr(sinr_db, rate_idx, freq);
	}

	return ber_sum / slot_count;
}

static double calc_rx_error_prob(struct wmediumd *ctx, struct station *sender,
				 struct station *receiver,
				 const struct timespec *ppdu_start,
				 const struct timespec *ppdu_end,
				 int signal_dbm, unsigned int rate_idx,
				 u32 freq, int frame_len, bool exclude_self)
{
	double signal_mw;
	double avg_ber;

	if (!receiver)
		return 0.0;

	signal_mw = dBm_to_milliwatt(signal_dbm);
	if (signal_mw <= 1.0)
		return 1.0;

	if (!rx_window_enabled(ctx) || !receiver->rx_window.slots)
		return ctx->get_error_prob(ctx, milliwatt_to_dBm(signal_mw), rate_idx,
					   freq, frame_len, sender, receiver);

	avg_ber = calc_avg_ber_from_window(receiver, ppdu_start, ppdu_end,
					   signal_mw, exclude_self,
					   rate_idx, freq);
	return get_error_prob_from_ber(avg_ber, rate_idx, freq, frame_len);
}

static int medium_interval_cmp(const void *lhs, const void *rhs)
{
	const struct medium_interval *a = lhs;
	const struct medium_interval *b = rhs;
	u64 a_start = timespec_to_usec(&a->start);
	u64 b_start = timespec_to_usec(&b->start);
	u64 a_end = timespec_to_usec(&a->end);
	u64 b_end = timespec_to_usec(&b->end);

	if (a_start < b_start)
		return -1;
	if (a_start > b_start)
		return 1;
	if (a_end < b_end)
		return -1;
	if (a_end > b_end)
		return 1;
	return 0;
}

static size_t collect_medium_intervals(struct wmediumd *ctx, struct station *station,
				       struct frame *skip,
				       const struct timespec *base_time,
				       struct medium_interval **out)
{
	struct station *peer;
	u64 base_us = timespec_to_usec(base_time);
	size_t count = 0;
	size_t idx = 0;
	int ac;

	*out = NULL;

	list_for_each_entry(peer, &ctx->stations, list) {
		if (!same_medium(station, peer))
			continue;

		for (ac = 0; ac < IEEE80211_NUM_ACS; ac++) {
			struct frame *queued;

			list_for_each_entry(queued, &peer->queues[ac].frames, list) {
				if (queued == skip)
					continue;
				if (!queued->attempt_scheduled)
					continue;
				if (timespec_to_usec(&queued->ppdu_end) <= base_us)
					continue;
				count++;
			}
		}
	}

	if (!count)
		return 0;

	*out = calloc(count, sizeof(**out));
	if (!*out)
		return 0;

	list_for_each_entry(peer, &ctx->stations, list) {
		if (!same_medium(station, peer))
			continue;

		for (ac = 0; ac < IEEE80211_NUM_ACS; ac++) {
			struct frame *queued;

			list_for_each_entry(queued, &peer->queues[ac].frames, list) {
				if (queued == skip)
					continue;
				if (!queued->attempt_scheduled)
					continue;
				if (timespec_to_usec(&queued->ppdu_end) <= base_us)
					continue;

				(*out)[idx].start = queued->ppdu_start;
				(*out)[idx].end = queued->ppdu_end;
				idx++;
			}
		}
	}

	qsort(*out, idx, sizeof(**out), medium_interval_cmp);
	return idx;
}

/*
 * 沿着同一 medium 中已预约的数据忙时段扫描，仿真 DCF 在信道忙时
 * 冻结退避计数器、空闲后等待 DIFS 再继续倒计时的行为。
 */
static struct timespec find_backoff_completion_time(struct wmediumd *ctx,
						    struct station *station,
						    struct frame *frame,
						    const struct timespec *base_time,
						    int difs_us, int slot_time_us,
						    int backoff_slots)
{
	struct medium_interval *intervals = NULL;
	size_t count;
	u64 cursor_us = timespec_to_usec(base_time);
	u64 backoff_us = (u64)backoff_slots * slot_time_us;
	bool need_difs = true;
	size_t i;

	count = collect_medium_intervals(ctx, station, frame, base_time, &intervals);
	for (i = 0; i < count; i++) {
		u64 start_us = timespec_to_usec(&intervals[i].start);
		u64 end_us = timespec_to_usec(&intervals[i].end);
		u64 idle_us;

		if (end_us <= cursor_us)
			continue;

		if (start_us <= cursor_us) {
			cursor_us = end_us;
			need_difs = true;
			continue;
		}

		idle_us = start_us - cursor_us;
		if (need_difs) {
			if (idle_us < (u64)difs_us) {
				cursor_us = end_us;
				continue;
			}

			cursor_us += difs_us;
			idle_us -= difs_us;
			need_difs = false;
		}

		if (backoff_us <= idle_us) {
			cursor_us += backoff_us;
			goto out;
		}

		backoff_us -= (idle_us / slot_time_us) * slot_time_us;
		cursor_us = end_us;
		need_difs = true;
	}

	if (need_difs)
		cursor_us += difs_us;
	cursor_us += backoff_us;

out:
	free(intervals);
	return timespec_from_usec(cursor_us);
}

static int random_backoff_slots(struct wmediumd *ctx, int cw)
{
	if (use_fixed_random_value(ctx))
		return cw / 2;

	return (int)(drand48() * (cw + 1));
}

static double draw_frame_choice(struct wmediumd *ctx, double fixed_choice)
{
	if (use_fixed_random_value(ctx))
		return fixed_choice;

	return drand48();
}

static void finalize_frame_tx_rates(struct frame *frame)
{
	int i;

	for (i = 0; i < frame->tx_rates_count; i++) {
		if (frame->tx_rates[i].idx < 0 || !frame->attempts_done[i]) {
			frame->tx_rates[i].idx = -1;
			frame->tx_rates[i].count = -1;
			continue;
		}

		frame->tx_rates[i].count = frame->attempts_done[i];
	}
}

static void shift_station_queued_frames(struct wmediumd *ctx, struct station *station,
					struct frame *skip, struct timespec *base,
					u64 delta_us)
{
	int ac;

	if (!delta_us)
		return;

	for (ac = 0; ac < IEEE80211_NUM_ACS; ac++) {
		struct frame *queued;

		list_for_each_entry(queued, &station->queues[ac].frames, list) {
			if (queued == skip)
				continue;
			if (!queued->attempt_scheduled)
				continue;
			if (timespec_before(&queued->ppdu_start, base))
				continue;

			release_ppdu_for_receivers(ctx, queued->sender,
						   &queued->ppdu_start,
						   &queued->ppdu_end);
			timespec_add_usec64(&queued->ppdu_start, delta_us);
			timespec_add_usec64(&queued->ppdu_end, delta_us);
			timespec_add_usec64(&queued->expires, delta_us);
			reserve_ppdu_for_receivers(ctx, queued->sender,
						   &queued->ppdu_start,
						   &queued->ppdu_end);
		}
	}
}

/*
 * 成功发送后的 ACK/SIFS 是整个 medium 共享的忙时段。发送端自己的后续
 * 队列已经串行化到 expires，无需再次平移；其余同一 medium 的未来帧
 * 需要整体后移，避免把 ACK 窗口误当作空闲信道。
 */
static void shift_medium_queued_frames(struct wmediumd *ctx, struct frame *frame,
				       struct timespec *base, u64 delta_us)
{
	struct station *station;
	int ac;

	if (!delta_us)
		return;

	list_for_each_entry(station, &ctx->stations, list) {
		if (!same_medium(frame->sender, station))
			continue;

		for (ac = 0; ac < IEEE80211_NUM_ACS; ac++) {
			struct frame *queued;

			list_for_each_entry(queued, &station->queues[ac].frames, list) {
				if (queued == frame)
					continue;
				if (!queued->attempt_scheduled)
					continue;
				if (timespec_before(&queued->ppdu_start, base))
					continue;
				if (queued->sender == frame->sender &&
				    !timespec_before(&queued->ppdu_start,
						     &frame->expires))
					continue;

				release_ppdu_for_receivers(ctx, queued->sender,
							   &queued->ppdu_start,
							   &queued->ppdu_end);
				timespec_add_usec64(&queued->ppdu_start, delta_us);
				timespec_add_usec64(&queued->ppdu_end, delta_us);
				timespec_add_usec64(&queued->expires, delta_us);
				reserve_ppdu_for_receivers(ctx, queued->sender,
							   &queued->ppdu_start,
							   &queued->ppdu_end);
			}
		}
	}
}

/*
 * 低节点区间里，若长期保留各站点已经算好的“未来队首预约”，
 * 启动阶段的错相会被一直继承下去，接收端看到的上行序列会过于有序。
 * 这里在每轮 busy 结束后重建同一 medium 的队首预约，让各站点重新
 * 从共同基准参与下一轮竞争。
 */
static int frame_has_more_attempts(struct frame *frame)
{
	int pos = frame->current_rate_pos;

	while (pos < frame->tx_rates_count) {
		if (frame->tx_rates[pos].idx < 0)
			return 0;
		if (frame->attempts_done[pos] < frame->tx_rates[pos].count)
			return 1;
		pos++;
	}

	return 0;
}

static void advance_frame_retry_state(struct frame *frame, struct wqueue *queue)
{
	frame->current_cw = (frame->current_cw << 1) + 1;
	if (frame->current_cw > queue->cw_max)
		frame->current_cw = queue->cw_max;

	while (frame->current_rate_pos < frame->tx_rates_count) {
		if (frame->tx_rates[frame->current_rate_pos].idx < 0)
			return;
		if (frame->attempts_done[frame->current_rate_pos] <
		    frame->tx_rates[frame->current_rate_pos].count)
			return;
		frame->current_rate_pos++;
	}
}

/*
 * 为当前帧只安排“一次”物理层发送尝试。是否成功留到 attempt 到期时，
 * 再根据届时的窗口重叠情况判定。
 */
static void schedule_frame_attempt(struct wmediumd *ctx, struct station *station,
				   struct frame *frame,
				   const struct timespec *base_time)
{
	int slot_time = 9;
	int sifs = 16;
	int difs = 2 * slot_time + sifs;
	int rate_idx;
	int airtime_us;
	int access_delay_us;
	int backoff_slots;
	int signal_dbm = SNR_DEFAULT + NOISE_LEVEL;
	struct station *deststa = NULL;
	struct ieee80211_hdr *hdr = (void *)frame->data;

	rate_idx = frame->tx_rates[frame->current_rate_pos].idx;
	backoff_slots = random_backoff_slots(ctx, frame->current_cw);
	airtime_us = pkt_duration(ctx, frame->data_len,
				      index_to_rate(rate_idx, frame->freq));

	frame->ppdu_start = find_backoff_completion_time(ctx, station, frame,
							 base_time, difs,
							 slot_time, backoff_slots);
	access_delay_us = timespec_to_usec(&frame->ppdu_start) -
			  timespec_to_usec(base_time);
	frame->ppdu_end = timespec_plus_usec(&frame->ppdu_start, airtime_us);
	frame->ppdu_airtime_us = airtime_us;
	frame->final_rate_idx = rate_idx;
	frame->signal = signal_dbm;
	frame->expires = frame->ppdu_end;
	if (!frame->noack)
		timespec_add_usec64(&frame->expires, frame->ack_exchange_us);
	frame->attempt_scheduled = true;

	frame->service_time_us += access_delay_us + airtime_us;
	if (!frame->noack)
		frame->service_time_us += frame->ack_exchange_us;

	if (!is_multicast_ether_addr(hdr->addr1))
		deststa = get_station_by_addr(ctx, hdr->addr1);

	if (deststa)
		frame->signal = get_signal_dbm(ctx, station, deststa, true);

	frame->attempts_done[frame->current_rate_pos]++;
	reserve_ppdu_for_receivers(ctx, station, &frame->ppdu_start, &frame->ppdu_end);
}

static int __attribute__((unused)) set_interference_duration(struct wmediumd *ctx,
				     int src_idx, int duration, int signal)
{
	int i, medium_id;

	if (!ctx->intf)
		return 0;

	if (signal >= CCA_THRESHOLD)
		return 0;

    medium_id = ctx->sta_array[src_idx]->medium_id;
	for (i = 0; i < ctx->num_stas; i++) {
        if (medium_id != ctx->sta_array[i]->medium_id)
            continue;
		ctx->intf[ctx->num_stas * src_idx + i].duration += duration;
		// use only latest value
		ctx->intf[ctx->num_stas * src_idx + i].signal = signal;
	}

	return 1;
}

static int get_signal_offset_by_interference(struct wmediumd *ctx, int src_idx,
					     int dst_idx)
{
    int i, medium_id;
	double intf_power;

	if (!ctx->intf)
		return 0;

	intf_power = 0.0;
    medium_id = ctx->sta_array[dst_idx]->medium_id;
	for (i = 0; i < ctx->num_stas; i++) {
		if (i == src_idx || i == dst_idx)
			continue;
        if (medium_id != ctx->sta_array[i]->medium_id)
            continue;
		if (drand48() < ctx->intf[i * ctx->num_stas + dst_idx].prob_col)
			intf_power += dBm_to_milliwatt(
				ctx->intf[i * ctx->num_stas + dst_idx].signal);
	}

	if (intf_power <= 1.0)
		return 0;

	return (int)(milliwatt_to_dBm(intf_power) + 0.5);
}

static bool is_multicast_ether_addr(const u8 *addr)
{
	return 0x01 & addr[0];
}

static struct station *get_station_by_addr(struct wmediumd *ctx, u8 *addr)
{
	struct station *station;

	list_for_each_entry(station, &ctx->stations, list) {
		if (memcmp(station->addr, addr, ETH_ALEN) == 0)
			return station;
	}
	return NULL;
}

static int frame_delivery_rate_idx(struct frame *frame)
{
	int i;
	int rate_idx = 0;

	if (frame->final_rate_idx >= 0)
		return frame->final_rate_idx;

	for (i = 0; i < frame->tx_rates_count; i++) {
		if (frame->tx_rates[i].idx < 0)
			break;
		rate_idx = frame->tx_rates[i].idx;
	}

	return rate_idx;
}

void detect_mediums(struct wmediumd *ctx, struct station *src, struct station *dest) {
    int medium_id;
    if (!ctx->enable_medium_detection){
        return;
    }
    if(src->isap& !dest->isap){
        // AP-STA Connection
        medium_id = -src->index-1;
    }else if((!src->isap)& dest->isap){
        // STA-AP Connection
        medium_id = -dest->index-1;
    }else{
        // AP-AP Connection
        // STA-STA Connection
        // TODO: Detect adhoc and mesh groups
        return;
    }
    if (medium_id!=src->medium_id){
        w_logf(ctx, LOG_DEBUG, "Setting medium id of " MAC_FMT "(%d|%s) to %d.\n",
               MAC_ARGS(src->addr), src->index, src->isap ? "AP" : "Sta",
               medium_id);
        src-> medium_id = medium_id;
    }
    if(medium_id!=dest->medium_id){
        w_logf(ctx, LOG_DEBUG, "Setting medium id of " MAC_FMT "(%d|%s) to %d.\n",
               MAC_ARGS(dest->addr), dest->index, dest->isap ? "AP" : "Sta",
               medium_id);
        dest-> medium_id = medium_id;
    }
}
void queue_frame(struct wmediumd *ctx, struct station *station,
		 struct frame *frame)
{
	struct ieee80211_hdr *hdr = (void *)frame->data;
	u8 *dest = hdr->addr1;
	struct timespec now, target;
	struct wqueue *queue;
	struct frame *tail;
	struct station *deststa;
	int ac;
	int ack_airtime_us;
	int i;
	bool queue_was_empty;

	clock_gettime(CLOCK_MONOTONIC, &now);

	ack_airtime_us = pkt_duration(ctx, 14, index_to_rate(0, frame->freq));

	ac = frame_select_queue_80211(frame);
	queue = &station->queues[ac];
	queue_was_empty = list_empty(&queue->frames);

	if (is_multicast_ether_addr(dest)) {
		deststa = NULL;
	} else {
		deststa = get_station_by_addr(ctx, dest);
		if (deststa) {
			w_logf(ctx, LOG_DEBUG,
			       "Packet from " MAC_FMT "(%d|%s) to " MAC_FMT "(%d|%s)\n",
			       MAC_ARGS(station->addr), station->index,
			       station->isap ? "AP" : "Sta",
			       MAC_ARGS(deststa->addr), deststa->index,
			       deststa->isap ? "AP" : "Sta");
			detect_mediums(ctx, station, deststa);
		}
	}

	frame->queue_ac = ac;
	frame->noack = frame_is_mgmt(frame) || is_multicast_ether_addr(dest);
	frame->fixed_choice = use_fixed_random_value(ctx) ? drand48() : -1.0;
	frame->flags &= ~HWSIM_TX_STAT_ACK;
	frame->service_time_us = 0;
	frame->ppdu_airtime_us = 0;
	frame->final_rate_idx = -1;
	frame->current_rate_pos = 0;
	frame->current_cw = queue->cw_min;
	frame->ack_exchange_us = 16 + ack_airtime_us;
	memset(frame->attempts_done, 0, sizeof(frame->attempts_done));
	clear_frame_schedule(frame);

	/*
	 * 若队列只是短暂见底，则沿用最近一次 medium 忙转闲时刻作为竞争
	 * 基准，避免 paced 业务把不同站点的回退相位长期错开。
	 */
	target = queue_was_empty ? recent_busy_sync_base(ctx, &now) : now;
	list_add_tail(&frame->list, &queue->frames);
	for (i = 0; i <= ac; i++) {
		tail = queue_head_frame(&station->queues[i]);
		if (tail && tail != frame && tail->attempt_scheduled &&
		    timespec_before(&target, &tail->expires))
			target = tail->expires;
	}
	schedule_queue_head_frame(ctx, station, ac, &target);
	rearm_timer(ctx);
}

/*
 * Report transmit status to the kernel.
 */
static int send_tx_info_frame_nl(struct wmediumd *ctx, struct frame *frame)
{
	struct nl_sock *sock = ctx->sock;
	struct nl_msg *msg;
	int ret;

	msg = nlmsg_alloc();
	if (!msg) {
		w_logf(ctx, LOG_ERR, "Error allocating new message MSG!\n");
		return -1;
	}

	if (genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, ctx->family_id,
			0, NLM_F_REQUEST, HWSIM_CMD_TX_INFO_FRAME,
			VERSION_NR) == NULL) {
		w_logf(ctx, LOG_ERR, "%s: genlmsg_put failed\n", __func__);
		ret = -1;
		goto out;
	}

	if (nla_put(msg, HWSIM_ATTR_ADDR_TRANSMITTER, ETH_ALEN,
		    frame->sender->hwaddr) ||
	    nla_put_u32(msg, HWSIM_ATTR_FLAGS, frame->flags) ||
	    nla_put_u32(msg, HWSIM_ATTR_SIGNAL, frame->signal) ||
	    nla_put(msg, HWSIM_ATTR_TX_INFO,
		    frame->tx_rates_count * sizeof(struct hwsim_tx_rate),
		    frame->tx_rates) ||
	    nla_put_u64(msg, HWSIM_ATTR_COOKIE, frame->cookie)) {
			w_logf(ctx, LOG_ERR, "%s: Failed to fill a payload\n", __func__);
			ret = -1;
			goto out;
	}

	ret = nl_send_auto_complete(sock, msg);
	if (ret < 0) {
		w_logf(ctx, LOG_ERR, "%s: nl_send_auto failed\n", __func__);
		ret = -1;
		goto out;
	}
	ret = 0;

out:
	nlmsg_free(msg);
	return ret;
}

/*
 * Report transmit status to the transmitter.
 */
static int send_tx_info_frame(struct wmediumd *ctx, struct frame *frame)
{
	if (ctx->op_mode == LOCAL)
		return send_tx_info_frame_nl(ctx, frame);
	
	int numBytes, ret;

	if (is_ap){
		numBytes = sendto_retry(ctx->net_sock, frame, sizeof(*frame), 0,
					 (struct sockaddr *)&clientAddr, len);
        if (numBytes < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to send xmit info to station: %s", strerror(errno));
			ret = -1;
			goto out;
		}
	}
	else{
		numBytes = sendto_retry(ctx->net_sock, frame, sizeof(*frame), 0,
					 (struct sockaddr *)&serverAddr, len);
        if (numBytes < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to send xmit info to AP station: %s", strerror(errno));
			ret = -1;
			goto out;
		}
	}
	ret = 0;

out:
	return ret;
}

/*
 * Send a data frame to the kernel for reception at a specific radio.
 */
int send_cloned_frame_msg(struct wmediumd *ctx, struct station *dst,
			  u8 *data, int data_len, int rate_idx, int signal,
			  int freq)
{
	struct nl_msg *msg;
	struct nl_sock *sock = ctx->sock;
	int ret;

	msg = nlmsg_alloc();
	if (!msg) {
		w_logf(ctx, LOG_ERR, "Error allocating new message MSG!\n");
		return -1;
	}

	if (genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, ctx->family_id,
			0, NLM_F_REQUEST, HWSIM_CMD_FRAME,
			VERSION_NR) == NULL) {
		w_logf(ctx, LOG_ERR, "%s: genlmsg_put failed\n", __func__);
		ret = -1;
		goto out;
	}

	if (nla_put(msg, HWSIM_ATTR_ADDR_RECEIVER, ETH_ALEN,
		    dst->hwaddr) ||
	    nla_put(msg, HWSIM_ATTR_FRAME, data_len, data) ||
	    nla_put_u32(msg, HWSIM_ATTR_RX_RATE, rate_idx) ||
	    nla_put_u32(msg, HWSIM_ATTR_FREQ, freq) ||
	    nla_put_u32(msg, HWSIM_ATTR_SIGNAL, signal)) {
			w_logf(ctx, LOG_ERR, "%s: Failed to fill a payload\n", __func__);
			ret = -1;
			goto out;
	}

	w_logf(ctx, LOG_DEBUG, "cloned msg dest " MAC_FMT " (radio: " MAC_FMT ") len %d\n",
		   MAC_ARGS(dst->addr), MAC_ARGS(dst->hwaddr), data_len);

	ret = nl_send_auto_complete(sock, msg);
	if (ret < 0) {
		w_logf(ctx, LOG_ERR, "%s: nl_send_auto failed\n", __func__);
		ret = -1;
		goto out;
	}
	ret = 0;

out:
	nlmsg_free(msg);
	return ret;
}

void deliver_frame(struct wmediumd *ctx, struct frame *frame)
{
	struct ieee80211_hdr *hdr = (void *) frame->data;
	struct station *station;
	struct station *deststa = NULL;
	struct wqueue *queue = &frame->sender->queues[frame->queue_ac];
	u8 *dest = hdr->addr1;
	u8 *src = frame->sender->addr;
	int rate_idx = frame_delivery_rate_idx(frame);
	double error_prob = 0.0;
	bool success = frame->noack;

	if (!frame->noack && !is_multicast_ether_addr(dest))
		deststa = get_station_by_addr(ctx, dest);

	if (!frame->noack && deststa) {
		if (rx_window_enabled(ctx)) {
			error_prob = calc_rx_error_prob(ctx, frame->sender, deststa,
							 &frame->ppdu_start,
							 &frame->ppdu_end,
							 frame->signal, rate_idx,
							 frame->freq,
							 frame->data_len, true);
		} else {
			int snr = ctx->get_link_snr(ctx, frame->sender, deststa) -
				get_signal_offset_by_interference(ctx,
					frame->sender->index, deststa->index);

			snr += ctx->get_fading_signal(ctx);
			frame->signal = snr + NOISE_LEVEL;
			error_prob = ctx->get_error_prob(ctx, snr, rate_idx,
							 frame->freq, frame->data_len,
							 frame->sender, deststa);
		}
		success = draw_frame_choice(ctx, frame->fixed_choice) > error_prob;
	}

	release_ppdu_for_receivers(ctx, frame->sender,
				   &frame->ppdu_start, &frame->ppdu_end);
	frame->attempt_scheduled = false;

	if (!success && !frame->noack) {
		struct timespec retry_base = frame->expires;

		if (timespec_before(&ctx->last_global_busy_end, &retry_base))
			ctx->last_global_busy_end = retry_base;

		advance_frame_retry_state(frame, queue);
		if (frame_has_more_attempts(frame)) {
			u64 extra_delay;

			schedule_frame_attempt(ctx, frame->sender, frame, &retry_base);
			extra_delay = timespec_to_usec(&frame->expires) -
				      timespec_to_usec(&retry_base);
			shift_station_queued_frames(ctx, frame->sender, frame,
						    &retry_base, extra_delay);
			list_add(&frame->list, &queue->frames);
			rearm_timer(ctx);
			return;
		}
	}

	if (success)
		frame->flags |= HWSIM_TX_STAT_ACK;

	if (success && !frame->noack)
		shift_medium_queued_frames(ctx, frame, &frame->ppdu_end,
					   frame->ack_exchange_us);

	if (frame->flags & HWSIM_TX_STAT_ACK) {
		/* rx the frame on the dest interface */
		list_for_each_entry(station, &ctx->stations, list) {
			if (memcmp(src, station->addr, ETH_ALEN) == 0)
				continue;

			if (is_multicast_ether_addr(dest)) {
				int snr, signal;
				double rx_error_prob;

				snr = ctx->get_link_snr(ctx, frame->sender, station);
				snr += ctx->get_fading_signal(ctx);
				signal = snr + NOISE_LEVEL;
				if (signal < CCA_THRESHOLD)
					continue;

				if (rx_window_enabled(ctx)) {
					/*
					 * medium_id 用于划分干扰域，不应阻断
					 * 广播/组播控制帧的实际可达性。
					 * 对目标接收端计算 SINR 时也需要扣除
					 * 当前帧自身的信号功率。
					 */
					rx_error_prob = calc_rx_error_prob(ctx, frame->sender,
									   station,
									   &frame->ppdu_start,
									   &frame->ppdu_end,
									   signal, rate_idx,
									   frame->freq,
									   frame->data_len,
									   true);
				} else {
					if (signal < CCA_THRESHOLD)
						continue;

					rx_error_prob = ctx->get_error_prob(ctx,
						(double)snr, rate_idx, frame->freq,
						frame->data_len, frame->sender,
						station);
				}

				if (drand48() <= rx_error_prob) {
					w_logf(ctx, LOG_INFO, "Dropped mcast from "
						   MAC_FMT " to " MAC_FMT " at receiver\n",
						   MAC_ARGS(src), MAC_ARGS(station->addr));
					continue;
				}

				send_cloned_frame_msg(ctx, station,
						      frame->data,
						      frame->data_len,
						      rate_idx, signal,
						      frame->freq);
			} else if (memcmp(dest, station->addr, ETH_ALEN) == 0) {
				send_cloned_frame_msg(ctx, station,
						      frame->data,
						      frame->data_len,
						      rate_idx, frame->signal,
						      frame->freq);
			}
		}
	}

	if (timespec_before(&ctx->last_global_busy_end, &frame->expires))
		ctx->last_global_busy_end = frame->expires;

	schedule_queue_head_frame(ctx, frame->sender, frame->queue_ac,
				  &frame->expires);
	finalize_frame_tx_rates(frame);
	send_tx_info_frame(ctx, frame);
	free(frame);
}

void deliver_expired_frames_queue(struct wmediumd *ctx,
				  struct list_head *queue,
				  struct timespec *now)
{
	struct frame *frame, *tmp;

	list_for_each_entry_safe(frame, tmp, queue, list) {
		if (timespec_before(&frame->expires, now)) {
			list_del(&frame->list);
			deliver_frame(ctx, frame);
		} else {
			break;
		}
	}
}

static void process_station_queues(struct wmediumd *ctx, struct station *station,
				   struct timespec *now)
{
	struct list_head *l;
	int i;
	int q_ct[IEEE80211_NUM_ACS] = {};

	for (i = 0; i < IEEE80211_NUM_ACS; i++) {
		list_for_each(l, &station->queues[i].frames)
			q_ct[i]++;
	}

	w_logf(ctx, LOG_DEBUG, "[" TIME_FMT "] Station " MAC_FMT
		       " BK %d BE %d VI %d VO %d\n",
	       TIME_ARGS(now), MAC_ARGS(station->addr),
	       q_ct[IEEE80211_AC_BK], q_ct[IEEE80211_AC_BE],
	       q_ct[IEEE80211_AC_VI], q_ct[IEEE80211_AC_VO]);

	for (i = 0; i < IEEE80211_NUM_ACS; i++)
		deliver_expired_frames_queue(ctx, &station->queues[i].frames, now);
}

void deliver_expired_frames(struct wmediumd *ctx)
{
	struct timespec now, _diff;
	struct station *station;
	struct list_head *rand_it, *rand_start;
	int i, j, duration;
	int sta1_medium_id;

	clock_gettime(CLOCK_MONOTONIC, &now);
	
	int rand_start_cnt = rand() % ctx->num_stas;
	
	// Find the randomized starting point in the list
	rand_start = &ctx->stations;
	for (i = 0; i < rand_start_cnt; i++) {
		rand_start = rand_start->next;
	}

	list_for_each(rand_it, rand_start) {
		if (rand_it == &ctx->stations)
			continue;
		station = list_entry(rand_it, struct station, list);
		process_station_queues(ctx, station, &now);
	}

	for (rand_it = ctx->stations.next; rand_it != rand_start;
	     rand_it = rand_it->next) {
		if (rand_it == &ctx->stations)
			continue;
		station = list_entry(rand_it, struct station, list);
		process_station_queues(ctx, station, &now);
	}
	w_logf(ctx, LOG_DEBUG, "\n\n");

	if (!ctx->intf)
		return;

	timespec_sub(&now, &ctx->intf_updated, &_diff);
	duration = (_diff.tv_sec * 1000000) + (_diff.tv_nsec / 1000);
	if (duration < 10000) // calc per 10 msec
		return;

	// update interference
	for (i = 0; i < ctx->num_stas; i++){
        sta1_medium_id = ctx->sta_array[i]->medium_id;
        for (j = 0; j < ctx->num_stas; j++) {
            if (i == j)
                continue;
            if (sta1_medium_id != ctx->sta_array[j]->medium_id)
                continue;
            // probability is used for next calc
            ctx->intf[i * ctx->num_stas + j].prob_col =
                    ctx->intf[i * ctx->num_stas + j].duration /
                    (double)duration;
            ctx->intf[i * ctx->num_stas + j].duration = 0;
        }
    }

	clock_gettime(CLOCK_MONOTONIC, &ctx->intf_updated);
}

static int process_recvd_data(struct wmediumd *ctx, struct nlmsghdr *nlh)
{
	struct nlattr *attrs[HWSIM_ATTR_MAX+1];
	/* generic netlink header*/
	struct genlmsghdr *gnlh = nlmsg_data(nlh);

	struct station *sender;
	struct frame *frame;
	struct ieee80211_hdr *hdr;
	u8 *src;

	if (gnlh->cmd == HWSIM_CMD_FRAME) {
		pthread_rwlock_rdlock(&snr_lock);
		/* we get the attributes*/
		genlmsg_parse(nlh, 0, attrs, HWSIM_ATTR_MAX, NULL);
		if (attrs[HWSIM_ATTR_ADDR_TRANSMITTER]) {
			u8 *hwaddr = (u8 *)nla_data(attrs[HWSIM_ATTR_ADDR_TRANSMITTER]);

			unsigned int data_len =
				nla_len(attrs[HWSIM_ATTR_FRAME]);
			char *data = (char *)nla_data(attrs[HWSIM_ATTR_FRAME]);
			unsigned int flags =
				nla_get_u32(attrs[HWSIM_ATTR_FLAGS]);
			unsigned int tx_rates_len =
				nla_len(attrs[HWSIM_ATTR_TX_INFO]);
			struct hwsim_tx_rate *tx_rates =
				(struct hwsim_tx_rate *)
				nla_data(attrs[HWSIM_ATTR_TX_INFO]);
			u64 cookie = nla_get_u64(attrs[HWSIM_ATTR_COOKIE]);
			u32 freq;
			freq = attrs[HWSIM_ATTR_FREQ] ?
					nla_get_u32(attrs[HWSIM_ATTR_FREQ]) : 2412;

			hdr = (struct ieee80211_hdr *)data;
			src = hdr->addr2;
			
			w_logf(ctx, LOG_DEBUG, "f: %02x%02x d: %02x%02x ",
					(u32)hdr->frame_control[0], (u32)hdr->frame_control[1], (u32)hdr->duration_id[0], (u32)hdr->duration_id[1]);
			
			if (data_len < 6 + 6 + 4)
				goto out;

			sender = get_station_by_addr(ctx, src);
			if (!sender) {
				w_flogf(ctx, LOG_ERR, stderr, "Unable to find sender station " MAC_FMT "\n", MAC_ARGS(src));
				goto out;
			}
			memcpy(sender->hwaddr, hwaddr, ETH_ALEN);

			frame = malloc(sizeof(*frame) + data_len);
			if (!frame)
				goto out;

			memcpy(frame->data, data, data_len);
			frame->data_len = data_len;
			frame->flags = flags;
			frame->cookie = cookie;
			frame->freq = freq;
			frame->sender = sender;
			sender->freq = freq;
			frame->tx_rates_count =
				tx_rates_len / sizeof(struct hwsim_tx_rate);
			memcpy(frame->tx_rates, tx_rates,
			       min(tx_rates_len, sizeof(frame->tx_rates)));
			
			w_logf(ctx, LOG_DEBUG, "a1: " MAC_FMT " a2: " MAC_FMT " a3: " MAC_FMT " sq: %02x%02x r: " MAC_FMT" len: %d cookie: %lld\n", 
					MAC_ARGS(hdr->addr1), MAC_ARGS(hdr->addr2), MAC_ARGS(hdr->addr3), (u32)hdr->seq_ctrl[0], (u32)hdr->seq_ctrl[1], 
					MAC_ARGS(frame->sender->hwaddr), data_len, cookie);
			
			queue_frame(ctx, sender, frame);
		}
out:
		pthread_rwlock_unlock(&snr_lock);
		return 0;

	}
	return 0;
}

static
int nl_err_cb(struct sockaddr_nl *nla, struct nlmsgerr *nlerr, void *arg)
{
	struct genlmsghdr *gnlh = nlmsg_data(&nlerr->msg);
	struct wmediumd *ctx = arg;

	w_flogf(ctx, LOG_ERR, stderr, "nl: cmd %d, seq %d: %s\n", gnlh->cmd,
			nlerr->msg.nlmsg_seq, strerror(abs(nlerr->error)));

	return NL_SKIP;
}

struct frame* construct_tx_info_frame(struct wmediumd *ctx, struct nlmsghdr *nlh)
{
	struct nlattr *attrs[HWSIM_ATTR_MAX+1];
	struct genlmsghdr *gnlh = nlmsg_data(nlh);

	struct station *sender;
	struct frame *frame = NULL;
	struct ieee80211_hdr *hdr;

	if (gnlh->cmd == HWSIM_CMD_FRAME){
		genlmsg_parse(nlh, 0, attrs, HWSIM_ATTR_MAX, NULL);
		u8 *hwaddr = (u8 *)nla_data(attrs[HWSIM_ATTR_ADDR_TRANSMITTER]);
		u64 cookie = nla_get_u64(attrs[HWSIM_ATTR_COOKIE]);
		char *data = (char *)nla_data(attrs[HWSIM_ATTR_FRAME]);

		hdr = (struct ieee80211_hdr *)data;

		sender = get_station_by_addr(ctx, hdr->addr2);
		if (!sender) {
			w_flogf(ctx, LOG_ERR, stderr, "%s: Unable to find sender station " MAC_FMT "\n", __FUNCTION__, MAC_ARGS(hdr->addr2));
			goto out;
		}
		memcpy(sender->hwaddr, hwaddr, ETH_ALEN);
		
		frame = malloc(sizeof(struct frame));
		if (!frame)
			goto out;
		
		frame->cookie = cookie;
		frame->sender = sender;
	}

out:
	return frame;
}

/*
 * Handle events from the kernel.  Process CMD_FRAME events and queue them
 * for later delivery with the scheduler.
 */
static int process_messages_cb(struct nl_msg *msg, void *arg)
{
	struct nlmsghdr *nlh = nlmsg_hdr(msg);
	struct wmediumd* ctx = (struct wmediumd*)arg;

	if (ctx->op_mode == LOCAL)
		return process_recvd_data(ctx, nlh);
	
	int numBytes, ret;

	out_buf = malloc(nlh->nlmsg_len);
	memcpy(out_buf, nlh, nlh->nlmsg_len);

	if (is_ap){
		numBytes = sendto_retry(ctx->net_sock, out_buf, nlh->nlmsg_len, 0,
					 (struct sockaddr *)&clientAddr, len);
        if (numBytes < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to send data to station: %s", strerror(errno));
			ret = -1;
			goto out;
		}
	}
	else{
		numBytes = sendto_retry(ctx->net_sock, out_buf, nlh->nlmsg_len, 0,
					 (struct sockaddr *)&serverAddr, len);
        if (numBytes < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to send data to AP station: %s", strerror(errno));
			ret = -1;
			goto out;
		}
	}
	ret = 0;
	struct frame* tx_frame = construct_tx_info_frame(ctx, nlh);
	if (tx_frame != NULL)
		list_add_tail(&tx_frame->list, &ctx->pending_txinfo_frames);

out:
	free(out_buf);
	return ret;
}

/*
 * Register with the kernel to start receiving new frames.
 */
int send_register_msg(struct wmediumd *ctx)
{
	struct nl_sock *sock = ctx->sock;
	struct nl_msg *msg;
	int ret;

	msg = nlmsg_alloc();
	if (!msg) {
		w_logf(ctx, LOG_ERR, "Error allocating new message MSG!\n");
		return -1;
	}

	if (genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, ctx->family_id,
			0, NLM_F_REQUEST, HWSIM_CMD_REGISTER,
			VERSION_NR) == NULL) {
		w_logf(ctx, LOG_ERR, "%s: genlmsg_put failed\n", __func__);
		ret = -1;
		goto out;
	}

	ret = nl_send_auto_complete(sock, msg);
	if (ret < 0) {
		w_logf(ctx, LOG_ERR, "%s: nl_send_auto failed\n", __func__);
		ret = -1;
		goto out;
	}
	ret = 0;

out:
	nlmsg_free(msg);
	return ret;
}

static void net_sock_event_cb(int fd, short what, void *data)
{
	struct wmediumd *ctx = data;
	int numBytes;
	struct frame* pending_frame;
	struct frame tx_info_frame;
	bzero(in_buf, PAGE_SIZE);

	if (is_ap){
		numBytes = recvfrom_retry(fd, in_buf, PAGE_SIZE, 0,
					  (struct sockaddr *)&clientAddr, &len);
        if (numBytes < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to receive data from station: %s", strerror(errno));
			return;
		} else if (numBytes == 0) {
			return;
		}
	}
	else{
		numBytes = recvfrom_retry(fd, in_buf, PAGE_SIZE, 0,
					  (struct sockaddr *)&serverAddr, &len);
        if (numBytes < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to receive data from AP station: %s", strerror(errno));
			return;
		} else if (numBytes == 0) {
			return;
		}
	}
	/* First check if the received message is for transmit status */
	memcpy(&tx_info_frame, in_buf, sizeof(struct frame));
	list_for_each_entry(pending_frame, &ctx->pending_txinfo_frames, list)
	{
		if (pending_frame->cookie == tx_info_frame.cookie){
			pending_frame->flags = tx_info_frame.flags;
			pending_frame->signal = tx_info_frame.signal;
			pending_frame->tx_rates_count = tx_info_frame.tx_rates_count;
			memcpy(pending_frame->tx_rates, tx_info_frame.tx_rates, 
					pending_frame->tx_rates_count * sizeof(struct hwsim_tx_rate));
			send_tx_info_frame_nl(ctx, pending_frame);
			list_del(&pending_frame->list);
			free(pending_frame);
			return;
		}
	}
	process_recvd_data(ctx, (struct nlmsghdr*)in_buf);
}

static void sock_event_cb(int fd, short what, void *data)
{
	struct wmediumd *ctx = data;

	nl_recvmsgs_default(ctx->sock);
}

/*
 * Setup netlink socket and callbacks.
 */
static int init_netlink(struct wmediumd *ctx)
{
	struct nl_sock *sock;
	int ret;

	ctx->cb = nl_cb_alloc(NL_CB_CUSTOM);
	if (!ctx->cb) {
		w_logf(ctx, LOG_ERR, "Error allocating netlink callbacks\n");
		return -1;
	}

	sock = nl_socket_alloc_cb(ctx->cb);
	if (!sock) {
		w_logf(ctx, LOG_ERR, "Error allocating netlink socket\n");
		return -1;
	}

	ctx->sock = sock;

	ret = genl_connect(sock);
	if (ret < 0) {
		w_logf(ctx, LOG_ERR, "Error connecting netlink socket ret=%d\n", ret);
		return -1;
	}

	ctx->family_id = genl_ctrl_resolve(sock, "MAC80211_HWSIM");
	if (ctx->family_id < 0) {
		w_logf(ctx, LOG_ERR, "Family MAC80211_HWSIM not registered\n");
		return -1;
	}

	nl_cb_set(ctx->cb, NL_CB_MSG_IN, NL_CB_CUSTOM, process_messages_cb, ctx);
	nl_cb_err(ctx->cb, NL_CB_CUSTOM, nl_err_cb, ctx);

	return 0;
}

int init_remote_connection(struct wmediumd* ctx, char* ap_ipaddr)
{
	int numBytes, ret = 0;

	ctx->net_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (ctx->net_sock < 0){
		w_flogf(ctx, LOG_ERR, stderr, "Failed to open socket: %s\n", strerror(errno));
		ret = -1;
		goto out;
	}
	configure_best_effort_socket(ctx->net_sock);
    
	bzero(&serverAddr, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(AP_DEFAULT_PORT);
	len = sizeof(struct sockaddr_in);

	/* AP server configuration */
	if (ap_ipaddr == NULL){
		serverAddr.sin_addr.s_addr = INADDR_ANY;
		if (bind(ctx->net_sock, (struct sockaddr *)&serverAddr, len) < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to bind socket: %s\n", strerror(errno));
			ret = -1;
			goto out;
		}
		w_logf(ctx, LOG_INFO, "Waiting for station machine to connect\n");
		
		/* Wait until station machine connects. This is primarily meant to get the client details */
		numBytes = recvfrom_retry(ctx->net_sock, in_buf, 5, 0,
					  (struct sockaddr *)&clientAddr, &len);
        if (numBytes < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to receive initiate command from client: %s\n", strerror(errno));
			ret = -1;
			goto out;
		}
		w_logf(ctx, LOG_NOTICE, "Station machine connected successfully\n");
	}
	else{
		struct hostent *server = NULL;
		server = gethostbyname(ap_ipaddr);
		if (server == NULL){
			w_flogf(ctx, LOG_ERR, stderr, "No host with the given hostname exists\n");
			ret = -1;
			goto out;
		}
		bcopy((char*)server->h_addr_list[0], (char*)&serverAddr.sin_addr.s_addr, server->h_length);
		
		/* The client must send a message first for the server to know the client address details */
		const char* initiate_comm = "start";
		numBytes = sendto_retry(ctx->net_sock, initiate_comm,
						strlen(initiate_comm), 0,
						(const struct sockaddr *)&serverAddr,
						len);
		if (numBytes < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to send initiate message: %s\n", strerror(errno));
			ret = -1;
			goto out;
		}
		w_logf(ctx, LOG_INFO, "Initiate communication command sent to access point machine\n");
	}

out:
	return ret;
}

/*
 *	Print the CLI help
 */
void print_help(int exval)
{
	printf("wmediumd v%s - a wireless medium simulator\n", VERSION_STR);
	printf("wmediumd [-h] [-V] [-a AP_ADDR] [-s] [-l LOG_LVL] [-x FILE] -c FILE\n\n");

	printf("  -h              print this help and exit\n");
	printf("  -V              print version and exit\n\n");

	printf("  -l LOG_LVL      set the logging level\n");
	printf("                  LOG_LVL: RFC 5424 severity, values 0 - 7\n");
	printf("                  >= 3: errors are logged\n");
	printf("                  >= 5: startup msgs are logged\n");
	printf("                  >= 6: dropped packets are logged (default)\n");
	printf("                  == 7: all packets will be logged\n");
	printf("  -c FILE         set input config file\n");
	printf("  -x FILE         set input PER file\n");
	printf("  -s              start the server on a socket\n");
	printf("  -d              use the dynamic complex mode\n");
	printf("                  (server only with matrices for each connection)\n");
	printf("  -a AP_ADDR      Set remote operation mode to associate remotely\n");
	printf("                  AP_ADDR: IPv4 address or hostname of AP machine\n");
	printf("                  \"localhost\" if current machine is AP\n");

	exit(exval);
}

static void timer_cb(int fd, short what, void *data)
{
	struct wmediumd *ctx = data;
	uint64_t u;

	pthread_rwlock_rdlock(&snr_lock);
	if (read(fd, &u, sizeof(u)) < 0) {
		pthread_rwlock_unlock(&snr_lock);
		return;
	}
	ctx->move_stations(ctx);
	deliver_expired_frames(ctx);
	rearm_timer(ctx);
	pthread_rwlock_unlock(&snr_lock);
}

static void free_all_stations(struct wmediumd *ctx)
{
	struct station *station, *tmp;

	list_for_each_entry_safe(station, tmp, &ctx->stations, list) {
		list_del(&station->list);
		station_free_resources(station);
		free(station);
	}

	free(ctx->sta_array);
	ctx->sta_array = NULL;
}

int main(int argc, char *argv[])
{
	int opt;
	struct event ev_cmd, net_ev;
	struct event ev_timer;
	struct wmediumd ctx;
	char *config_file = NULL;
	char *per_file = NULL;
	char* ap_ip = NULL;

	setvbuf(stdout, NULL, _IOLBF, BUFSIZ);

	if (argc == 1) {
		fprintf(stderr, "This program needs arguments....\n\n");
		print_help(EXIT_FAILURE);
	}

	ctx.log_lvl = 6;
	unsigned long int parse_log_lvl;
	char* parse_end_token;
	bool start_server = false;
	bool full_dynamic = false;

	while ((opt = getopt(argc, argv, "hVc:l:x:sda:")) != -1) {
		switch (opt) {
		case 'h':
			print_help(EXIT_SUCCESS);
			break;
		case 'V':
			printf("wmediumd v%s - a wireless medium simulator "
			       "for mac80211_hwsim\n", VERSION_STR);
			exit(EXIT_SUCCESS);
			break;
		case 'c':
			config_file = optarg;
			break;
		case 'x':
			printf("Input packet error rate file: %s\n", optarg);
			per_file = optarg;
			break;
		case ':':
			printf("wmediumd: Error - Option `%c' "
			       "needs a value\n\n", optopt);
			print_help(EXIT_FAILURE);
			break;
		case 'l':
			parse_log_lvl = strtoul(optarg, &parse_end_token, 10);
			if ((parse_log_lvl == ULONG_MAX && errno == ERANGE) ||
			     optarg == parse_end_token || parse_log_lvl > 7) {
				printf("wmediumd: Error - Invalid RFC 5424 severity level: "
							   "%s\n\n", optarg);
				print_help(EXIT_FAILURE);
			}
			ctx.log_lvl = parse_log_lvl;
			break;
		case 'd':
			full_dynamic = true;
			break;
		case 's':
			start_server = true;
			break;
		case 'a':
			ap_ip = optarg;
			ctx.op_mode = REMOTE;
			is_ap = strcmp(ap_ip, "localhost") == 0;
			break;
		case '?':
			printf("wmediumd: Error - No such option: "
			       "`%c'\n\n", optopt);
			print_help(EXIT_FAILURE);
			break;
		}

	}

	if (optind < argc)
		print_help(EXIT_FAILURE);

	if (ap_ip == NULL)
		ctx.op_mode = LOCAL;

	if (full_dynamic) {
		if (config_file) {
			printf("%s: cannot use dynamic complex mode with config file\n", argv[0]);
			print_help(EXIT_FAILURE);
		}

		if (!start_server) {
			printf("%s: dynamic complex mode requires the server option\n", argv[0]);
			print_help(EXIT_FAILURE);
		}

		w_logf(&ctx, LOG_NOTICE, "Using dynamic complex mode instead of config file\n");
	} else {
		if (!config_file) {
			printf("%s: config file must be supplied\n", argv[0]);
			print_help(EXIT_FAILURE);
		}

		w_logf(&ctx, LOG_NOTICE, "Input configuration file: %s\n", config_file);
	}
	INIT_LIST_HEAD(&ctx.stations);
	if (load_config(&ctx, config_file, per_file, full_dynamic))
		return EXIT_FAILURE;

	/* init libevent */
	event_init();

	if (ctx.op_mode == REMOTE){
		INIT_LIST_HEAD(&ctx.pending_txinfo_frames);
		if (init_remote_connection(&ctx, is_ap ? NULL : ap_ip) < 0)
			return EXIT_FAILURE;
		event_set(&net_ev, ctx.net_sock, EV_READ | EV_PERSIST, net_sock_event_cb, &ctx);
		event_add(&net_ev, NULL);
	}

	/* init netlink */
	if (init_netlink(&ctx) < 0)
		return EXIT_FAILURE;

	event_set(&ev_cmd, nl_socket_get_fd(ctx.sock), EV_READ | EV_PERSIST,
		  sock_event_cb, &ctx);
	event_add(&ev_cmd, NULL);

	/* setup timers */
	ctx.timerfd = timerfd_create(CLOCK_MONOTONIC, 0);
	clock_gettime(CLOCK_MONOTONIC, &ctx.intf_updated);
	clock_gettime(CLOCK_MONOTONIC, &ctx.next_move);
	ctx.last_global_busy_end.tv_sec = 0;
	ctx.last_global_busy_end.tv_nsec = 0;
	ctx.next_move.tv_sec += MOVE_INTERVAL;
	event_set(&ev_timer, ctx.timerfd, EV_READ | EV_PERSIST, timer_cb, &ctx);
	event_add(&ev_timer, NULL);

	/* register for new frames */
	if (send_register_msg(&ctx) == 0) {
		w_logf(&ctx, LOG_NOTICE, "REGISTER SENT!\n");
	}

	if (start_server == true)
		start_wserver(&ctx);

	/* enter libevent main loop */
	event_dispatch();

	if (start_server == true)
		stop_wserver();

	free(ctx.sock);
	free(ctx.cb);
	free(ctx.intf);
	free(ctx.per_matrix);
	free_all_stations(&ctx);

	return EXIT_SUCCESS;
}
