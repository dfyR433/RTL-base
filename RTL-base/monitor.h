/**
 * @file  monitor.h
 * @brief Wi-Fi monitor-mode capture engine — RTL8721Dx
 */

#ifndef MONITOR_H
#define MONITOR_H

#include <stdint.h>
#include <stddef.h>
#include "FreeRTOS.h"

/* =========================================================================
 * Output format (compile-time, -DMONITOR_FORMAT=)
 * ========================================================================= */
#define MONITOR_FMT_CAP     0
#define MONITOR_FMT_PCAP    1
#define MONITOR_FMT_PCAPNG  2
#ifndef MONITOR_FORMAT
#  define MONITOR_FORMAT  MONITOR_FMT_PCAPNG
#endif
#if (MONITOR_FORMAT != MONITOR_FMT_CAP)  && \
    (MONITOR_FORMAT != MONITOR_FMT_PCAP) && \
    (MONITOR_FORMAT != MONITOR_FMT_PCAPNG)
#  error "MONITOR_FORMAT must be 0 (CAP), 1 (PCAP), or 2 (PCAPNG)"
#endif

/* =========================================================================
 * Wi-Fi interface index
 * ========================================================================= */
#ifndef WLAN_IDX
#  define WLAN_IDX  STA_WLAN_INDEX
#endif

/* =========================================================================
 * Channel hopping — adaptive dwell
 * ========================================================================= */
#define CHANNEL_LIST_LEN       38u
#define HOP_DWELL_MIN_MS       50u
#define HOP_DWELL_MAX_MS       300u
#define HOP_TRAFFIC_HIGH_THR   20u

/* =========================================================================
 * Task parameters
 * ========================================================================= */
#define MONITOR_TASK_STACK       4096u
#define HOP_TASK_STACK           1024u
#define WRITER_TASK_STACK        2048u
#define MONITOR_TASK_PRIO        (tskIDLE_PRIORITY + 3u)
#define HOP_TASK_PRIO            (tskIDLE_PRIORITY + 1u)
#define WRITER_TASK_PRIO         (tskIDLE_PRIORITY + 2u)
#define MONITOR_JOIN_TIMEOUT_MS  2000u
#define HOP_JOIN_TIMEOUT_MS      500u
#define WRITER_JOIN_TIMEOUT_MS   500u

/* =========================================================================
 * Frame pool — ISR path uses this
 * ========================================================================= */
#ifndef PACKET_POOL_SIZE
#  define PACKET_POOL_SIZE   64u
#endif
#define PACKET_BUFFER_SIZE   2346u
#define RING_SIZE            1024u

/* =========================================================================
 * Block pool — writer path uses this; avoids per-packet heap malloc
 * ========================================================================= */
#define BLOCK_POOL_SIZE      32u
#define BLOCK_MAX_SIZE       2448u

/* =========================================================================
 * Writer queue depth
 * ========================================================================= */
#ifndef WRITER_QUEUE_LEN
#  define WRITER_QUEUE_LEN   128u
#endif

/* =========================================================================
 * MAC and BPF filter limits
 * ========================================================================= */
#ifndef MAX_MAC_ENTRIES
#  define MAX_MAC_ENTRIES     16u
#endif
#ifndef MAX_BPF_RULES
#  define MAX_BPF_RULES       8u
#endif

typedef struct {
    uint16_t offset;   /**< Byte offset from 802.11 header  */
    uint8_t  mask;     /**< Bit mask                        */
    uint8_t  value;    /**< Expected value after mask       */
} monitor_bpf_rule_t;

/* =========================================================================
 * Per-channel statistics — Welford RSSI variance + traffic EMA
 * ========================================================================= */
#define RSSI_HIST_BINS  8u

typedef struct {
    uint32_t frames_total;
    uint32_t frames_mgmt;
    uint32_t frames_ctrl;
    uint32_t frames_data;
    uint32_t frames_other;
    int8_t   rssi_min;
    int8_t   rssi_max;
    int32_t  rssi_sum;
    int64_t  rssi_m2;
    uint32_t rssi_histogram[RSSI_HIST_BINS];
    uint32_t traffic_ema;
} channel_stats_t;

/* =========================================================================
 * Wire-format constants
 * ========================================================================= */
#define LINKTYPE_IEEE802_11           105u
#define LINKTYPE_IEEE802_11_RADIOTAP  127u
#define PCAP_MAGIC_US                 0xA1B2C3D4UL
#define IDB_OPT_TSRESOL               9u
#define IDB_OPT_FCSLEN                13u
#define IDB_TSRESOL_NS                9u

#ifndef IDB_FCS_LEN_BYTES
#  define IDB_FCS_LEN_BYTES           0u
#endif

/* =========================================================================
 * Global statistics — read-only from any context
 * ========================================================================= */
extern volatile uint32_t stats_captured;
extern volatile uint32_t stats_dropped_ring;
extern volatile uint32_t stats_dropped_pool;
extern volatile uint32_t stats_dropped_block;
extern volatile uint32_t stats_peak_pool_used;

/* =========================================================================
 * Filter types
 * ========================================================================= */
typedef enum {
    FILTER_ALL, FILTER_DATA, FILTER_MANAGEMENT, FILTER_CONTROL,
    FILTER_BEACON, FILTER_PROBE_REQ, FILTER_PROBE_RSP
} monitor_filter_t;

typedef enum {
    MAC_FILTER_NONE, MAC_FILTER_ALLOWLIST, MAC_FILTER_DENYLIST
} mac_filter_mode_t;

/* =========================================================================
 * Output sink — called from writer_task (safe for blocking I/O)
 * ========================================================================= */
typedef void (*monitor_sink_cb_t)(const void *data, size_t len);

/* =========================================================================
 * Per-frame capture callback — called from monitor_task (safe for OS calls)
 * ========================================================================= */
typedef void (*monitor_capture_cb_t)(const uint8_t *frame, uint32_t len,
                                      uint8_t channel, int8_t rssi,
                                      uint64_t ts_ns);

/* =========================================================================
 * Public API
 * ========================================================================= */
void monitor_start(void);
void monitor_stop(void);

void    monitor_set_fixed_channel(uint8_t ch);
void    monitor_set_hopping(void);
uint8_t monitor_get_channel(void);

void monitor_set_output_sink(monitor_sink_cb_t cb);

void monitor_set_filter(monitor_filter_t filter);
void monitor_set_rssi_threshold(int8_t min_rssi);

void monitor_set_mac_filter_mode(mac_filter_mode_t mode);
int  monitor_add_mac_filter   (const uint8_t *mac);
int  monitor_remove_mac_filter(const uint8_t *mac);
void monitor_clear_mac_filter (void);

void monitor_set_bpf_rules  (const monitor_bpf_rule_t *rules, uint8_t count);
void monitor_clear_bpf_rules(void);

int  monitor_get_channel_stats  (uint8_t channel, channel_stats_t *out);
void monitor_reset_channel_stats(void);

void monitor_set_capture_callback(monitor_capture_cb_t cb);

void monitor_get_stats  (uint32_t *captured, uint32_t *dropped_ring,
                         uint32_t *dropped_pool, uint32_t *peak_pool,
                         uint32_t *dropped_block);
void monitor_reset_stats(void);

#endif /* MONITOR_H */
