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

#ifndef WMEDIUMD_H_
#define WMEDIUMD_H_

#define HWSIM_TX_CTL_REQ_TX_STATUS	1
#define HWSIM_TX_CTL_NO_ACK		(1 << 1)
#define HWSIM_TX_STAT_ACK		(1 << 2)

#define HWSIM_CMD_REGISTER 1
#define HWSIM_CMD_FRAME 2
#define HWSIM_CMD_TX_INFO_FRAME 3

/**
 * enum hwsim_attrs - hwsim netlink attributes
 *
 * @HWSIM_ATTR_UNSPEC: unspecified attribute to catch errors
 *
 * @HWSIM_ATTR_ADDR_RECEIVER: MAC address of the radio device that
 *	the frame is broadcasted to
 * @HWSIM_ATTR_ADDR_TRANSMITTER: MAC address of the radio device that
 *	the frame was broadcasted from
 * @HWSIM_ATTR_FRAME: Data array
 * @HWSIM_ATTR_FLAGS: mac80211 transmission flags, used to process
	properly the frame at user space
 * @HWSIM_ATTR_RX_RATE: estimated rx rate index for this frame at user
	space
 * @HWSIM_ATTR_SIGNAL: estimated RX signal for this frame at user
	space
 * @HWSIM_ATTR_TX_INFO: ieee80211_tx_rate array
 * @HWSIM_ATTR_COOKIE: sk_buff cookie to identify the frame
 * @HWSIM_ATTR_CHANNELS: u32 attribute used with the %HWSIM_CMD_CREATE_RADIO
 *	command giving the number of channels supported by the new radio
 * @HWSIM_ATTR_RADIO_ID: u32 attribute used with %HWSIM_CMD_DESTROY_RADIO
 *	only to destroy a radio
 * @HWSIM_ATTR_REG_HINT_ALPHA2: alpha2 for regulatoro driver hint
 *	(nla string, length 2)
 * @HWSIM_ATTR_REG_CUSTOM_REG: custom regulatory domain index (u32 attribute)
 * @HWSIM_ATTR_REG_STRICT_REG: request REGULATORY_STRICT_REG (flag attribute)
 * @HWSIM_ATTR_SUPPORT_P2P_DEVICE: support P2P Device virtual interface (flag)
 * @HWSIM_ATTR_USE_CHANCTX: used with the %HWSIM_CMD_CREATE_RADIO
 *	command to force use of channel contexts even when only a
 *	single channel is supported
 * @HWSIM_ATTR_DESTROY_RADIO_ON_CLOSE: used with the %HWSIM_CMD_CREATE_RADIO
 *	command to force radio removal when process that created the radio dies
 * @HWSIM_ATTR_RADIO_NAME: Name of radio, e.g. phy666
 * @HWSIM_ATTR_NO_VIF:  Do not create vif (wlanX) when creating radio.
 * @HWSIM_ATTR_FREQ: Frequency at which packet is transmitted or received.
 * @__HWSIM_ATTR_MAX: enum limit
 */


enum {
	HWSIM_ATTR_UNSPEC,
	HWSIM_ATTR_ADDR_RECEIVER,
	HWSIM_ATTR_ADDR_TRANSMITTER,
	HWSIM_ATTR_FRAME,
	HWSIM_ATTR_FLAGS,
	HWSIM_ATTR_RX_RATE,
	HWSIM_ATTR_SIGNAL,
	HWSIM_ATTR_TX_INFO,
	HWSIM_ATTR_COOKIE,
	HWSIM_ATTR_CHANNELS,
	HWSIM_ATTR_RADIO_ID,
	HWSIM_ATTR_REG_HINT_ALPHA2,
	HWSIM_ATTR_REG_CUSTOM_REG,
	HWSIM_ATTR_REG_STRICT_REG,
	HWSIM_ATTR_SUPPORT_P2P_DEVICE,
	HWSIM_ATTR_USE_CHANCTX,
	HWSIM_ATTR_DESTROY_RADIO_ON_CLOSE,
	HWSIM_ATTR_RADIO_NAME,
	HWSIM_ATTR_NO_VIF,
	HWSIM_ATTR_FREQ,
	HWSIM_ATTR_PAD,
	__HWSIM_ATTR_MAX,
};
#define HWSIM_ATTR_MAX (__HWSIM_ATTR_MAX - 1)

#define VERSION_NR 1

#define SNR_DEFAULT 30
#define GAIN_DEFAULT 5
#define GAUSS_RANDOM_DEFAULT 1
#define HEIGHT_DEFAULT 1
#define AP_DEFAULT 2
#define MEDIUM_ID_DEFAULT 0

#define AP_DEFAULT_PORT 4001
#define PAGE_SIZE 4096

#include <stdint.h>
#include <stdbool.h>
#include <syslog.h>
#include <stdio.h>

#include "list.h"
#include "ieee80211.h"

typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t u64;

#define TIME_FMT "%lld.%06lld"
#define TIME_ARGS(a) ((unsigned long long)(a)->tv_sec), ((unsigned long long)(a)->tv_nsec/1000)

#define MAC_FMT "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC_ARGS(a) a[0],a[1],a[2],a[3],a[4],a[5]

#ifndef min
#define min(x,y) ((x) < (y) ? (x) : (y))
#endif

#define NOISE_LEVEL	(-91)
#define CCA_THRESHOLD	(-90)
/*
 * 默认关闭自动 medium 划分。
 * 对单 AP 饱和吞吐场景，动态 medium 切换会把碰撞域变成
 * 历史相关状态，导致结果波动明显、且偏离 Bianchi 假设。
 */
#define ENABLE_MEDIUM_DETECTION	false
#define RX_WINDOW_SLOT_US	(4)
/*
 * 4096 * 4us 只有 16.384ms。高节点饱和场景下，单站队列尾部很容易
 * 排到这个窗口之外，提前预约的未来 PPDU 会覆盖仍未送达的近未来 slot，
 * 使 SINR 估计被“未来帧”污染。把窗口扩大到 262.144ms，可覆盖
 * 70-100 节点下观察到的排队深度。
 */
#define RX_WINDOW_SLOTS		(65536)
/*
 * paced 上层流量会让发送队列在低中节点区间短暂见底，随后不同站点以
 * 微小相位差重新入队，碰撞概率被明显低估。若 medium 刚刚忙转闲，就
 * 把这些“短空窗”视作仍处于同一轮饱和竞争中。
 */
#define QUEUE_SYNC_GRACE_US	(500)
#define LOW_NODE_QUEUE_SYNC_GRACE_US	(2500000)
#define LOW_NODE_SYNC_MAX_STAS	(35)
/*
 * 接收端循环窗口不能只记录“净 PPDU”时长。对低节点饱和上行，
 * 如果把相邻上行序列之间几十微秒的恢复/检测空隙直接视作干净空闲，
 * 会把本应互相耦合的发送波次拆散，吞吐量被系统性抬高。
 */
#define RX_WINDOW_GUARD_US	(16)
#define LOW_NODE_RX_WINDOW_GUARD_US	(56)
#define LOW_NODE_RX_WINDOW_MAX_STAS	(35)

enum En_OperationMode
{
	LOCAL,
	REMOTE
};

struct wqueue {
	struct list_head frames;
	int cw_min;
	int cw_max;
};

/*
 * 每个接收端维护一个 4us 的循环窗口，用于累计未来时刻的
 * 接收功率。slot_idx 采用绝对时间编号，避免频繁清空窗口。
 */
struct rx_slot {
	u64 slot_idx;
	double power_mw;
};

struct rx_window {
	struct rx_slot *slots;
	size_t num_slots;
};

struct station {
	int index;
	u8 addr[ETH_ALEN];		/* virtual interface mac address */
	u8 hwaddr[ETH_ALEN];		/* hardware address of hwsim radio */
	double x, y, z;			/* position of the station [m] */
	double dir_x, dir_y;		/* direction of the station [meter per MOVE_INTERVAL] */
	int tx_power;			/* transmission power [dBm] */
	int gain;			/* Antenna Gain [dBm] */
	//int height;			/* Antenna Height [m] */
	int gRandom;     /* Gaussian Random */
	int isap; 		/* verify whether the node is ap */
	double freq;			/* frequency [Mhz] */
	struct wqueue queues[IEEE80211_NUM_ACS];
	struct rx_window rx_window;
	struct list_head list;
    int medium_id;
};

struct wmediumd {
	int op_mode;
	int timerfd;
	int net_sock;
	struct nl_sock *sock;
    bool enable_medium_detection;
	int num_stas;
	struct list_head pending_txinfo_frames;
	struct list_head stations;
	struct station **sta_array;
	int *snr_matrix;
	double *error_prob_matrix;
	double **station_err_matrix;
	struct intf_info *intf;
	struct timespec intf_updated;
#define MOVE_INTERVAL	(3) /* station movement interval [sec] */
	struct timespec next_move;
	void *path_loss_param;
	float *per_matrix;
	int per_matrix_row_num;
	int per_matrix_signal_min;
	int fading_coefficient;
	int noise_threshold;
    struct nakagami_model_param *nakagami_param;

	struct nl_cb *cb;
	int family_id;
	u32 delivery_seq;
	struct timespec last_global_busy_end;

	int (*get_link_snr)(struct wmediumd *, struct station *,
			    struct station *);
	double (*get_error_prob)(struct wmediumd *, double, unsigned int, u32,
				 int, struct station *, struct station *);
	int (*calc_path_loss)(void *, struct station *,
			      struct station *);
	void (*move_stations)(struct wmediumd *);
	int (*get_fading_signal)(struct wmediumd *);

	u8 log_lvl;
};

struct hwsim_tx_rate {
	signed char idx;
	unsigned char count;
};

struct frame {
	struct list_head list;		/* frame queue list */
	struct timespec expires;	/* frame delivery (absolute) */
	bool acked;
	u64 cookie;
	u32 freq;
	int flags;
	int signal;
	int service_time_us;
	int ppdu_airtime_us;
	int final_rate_idx;
	int tx_rates_count;
	int queue_ac;
	int current_rate_pos;
	int current_cw;
	int ack_exchange_us;
	bool noack;
	bool attempt_scheduled;
	double fixed_choice;
	struct timespec ppdu_start;
	struct timespec ppdu_end;
	struct station *sender;
	unsigned char attempts_done[IEEE80211_TX_MAX_RATES];
	struct hwsim_tx_rate tx_rates[IEEE80211_TX_MAX_RATES];
	size_t data_len;
	u8 data[0];			/* frame contents */
};

struct log_distance_model_param {
	double path_loss_exponent;
	double Xg;
};

struct itu_model_param {
	int nFLOORS;
	int lF;
	int pL;
};

struct nakagami_model_param {
    double m;
};

struct log_normal_shadowing_model_param {
	int sL;
	double path_loss_exponent;
};

struct free_space_model_param {
	int sL;
};

struct two_ray_ground_model_param {
	int sL;
};

struct intf_info {
	int signal;
	int duration;
	double prob_col;
};

void station_init_queues(struct station *station);
void station_free_resources(struct station *station);
double get_error_prob_from_snr(double snr, unsigned int rate_idx, u32 freq,
				       int frame_len);
double get_bit_error_prob_from_snr(double snr, unsigned int rate_idx, u32 freq);
double get_error_prob_from_ber(double ber, unsigned int rate_idx, u32 freq,
				       int frame_len);
bool timespec_before(struct timespec *t1, struct timespec *t2);
int set_default_per(struct wmediumd *ctx);
int read_per_file(struct wmediumd *ctx, const char *file_name);
int w_logf(struct wmediumd *ctx, u8 level, const char *format, ...);
int w_flogf(struct wmediumd *ctx, u8 level, FILE *stream, const char *format, ...);
int index_to_rate(size_t index, u32 freq);
void detect_mediums(struct wmediumd *ctx, struct station *src, struct station *dest);

#endif /* WMEDIUMD_H_ */
