#ifndef MONITOR_H
#define MONITOR_H

#include <stdint.h>
#include <stddef.h>
#include "FreeRTOS.h"

/* =========================================================================
 * Output format
 *
 *   MONITOR_FMT_CAP    – classic libpcap, raw IEEE 802.11 (no radiotap)
 *                        linktype 105.
 *
 *   MONITOR_FMT_PCAP   – classic libpcap, IEEE 802.11 + radiotap header
 *                        linktype 127.
 *
 *   MONITOR_FMT_PCAPNG – pcapng next-generation blocks, IEEE 802.11 +
 *                        radiotap. Nanosecond timestamps. (default)
 *
 * Override via compiler flag:
 *   CFLAGS += -DMONITOR_FORMAT=MONITOR_FMT_PCAP
 * ========================================================================= */

#define MONITOR_FMT_CAP     0
#define MONITOR_FMT_PCAP    1
#define MONITOR_FMT_PCAPNG  2

#ifndef MONITOR_FORMAT
#  define MONITOR_FORMAT    MONITOR_FMT_PCAPNG
#endif

#if (MONITOR_FORMAT != MONITOR_FMT_CAP)   && \
    (MONITOR_FORMAT != MONITOR_FMT_PCAP)  && \
    (MONITOR_FORMAT != MONITOR_FMT_PCAPNG)
#  error "MONITOR_FORMAT must be MONITOR_FMT_CAP, MONITOR_FMT_PCAP, or MONITOR_FMT_PCAPNG"
#endif

/* =========================================================================
 * Wi-Fi interface
 * ========================================================================= */
#ifndef WLAN_IDX
#  define WLAN_IDX  STA_WLAN_INDEX
#endif

/* =========================================================================
 * Channel hopping — 2.4 GHz + 5 GHz
 * ========================================================================= */
static const uint8_t CHANNEL_LIST[] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
    36, 40, 44, 48, 52, 56, 60, 64,
    100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
    149, 153, 157, 161, 165
};
#define CHANNEL_LIST_LEN  (sizeof(CHANNEL_LIST) / sizeof(CHANNEL_LIST[0]))
#define HOP_INTERVAL_MS   100

/* =========================================================================
 * Task stack sizes (bytes) and priorities
 * ========================================================================= */
#define MONITOR_TASK_STACK  4096
#define HOP_TASK_STACK      1024
#define WRITER_TASK_STACK   2048

#define MONITOR_TASK_PRIO   (tskIDLE_PRIORITY + 3)
#define HOP_TASK_PRIO       (tskIDLE_PRIORITY + 1)
#define WRITER_TASK_PRIO    (tskIDLE_PRIORITY + 2)

/* =========================================================================
 * Packet pool and ring buffer sizing
 * ========================================================================= */
#define RING_SIZE           1024
#ifndef PACKET_POOL_SIZE
#  define PACKET_POOL_SIZE  64
#endif
#define PACKET_BUFFER_SIZE  2346

/* =========================================================================
 * Wire-format constants
 * ========================================================================= */
#define LINKTYPE_IEEE802_11           105
#define LINKTYPE_IEEE802_11_RADIOTAP  127

#define PCAP_MAGIC_US  0xA1B2C3D4UL

/* pcapng IDB option codes (pcapng spec §4.2) */
#define IDB_OPT_TSRESOL    9
#define IDB_OPT_FCSLEN    13
#define IDB_TSRESOL_NS     9
#define IDB_FCS_LEN_BYTES  4

/* =========================================================================
 * Statistics — extern for external debug/telemetry access
 * ========================================================================= */
extern volatile uint32_t stats_captured;
extern volatile uint32_t stats_dropped_ring;
extern volatile uint32_t stats_dropped_pool;
extern volatile uint32_t stats_peak_pool_used;

/* =========================================================================
 * Types
 * ========================================================================= */
typedef enum {
    FILTER_ALL,         /**< All frames (default)            */
    FILTER_DATA,        /**< Data frames only (type = 2)     */
    FILTER_MANAGEMENT,  /**< Management frames (type = 0)    */
    FILTER_CONTROL,     /**< Control frames (type = 1)       */
    FILTER_BEACON,      /**< Beacon — management subtype 8   */
    FILTER_PROBE_REQ,   /**< Probe request — subtype 4       */
    FILTER_PROBE_RSP    /**< Probe response — subtype 5      */
} monitor_filter_t;

typedef enum {
    MAC_FILTER_NONE,       /**< No MAC filtering (default)    */
    MAC_FILTER_ALLOWLIST,  /**< Only pass frames from list    */
    MAC_FILTER_DENYLIST    /**< Drop frames from list         */
} mac_filter_mode_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

/* ── Lifecycle ─────────────────────────────────────────────────────────── */
void monitor_start(void);
void monitor_stop(void);

/* ── Channel control ───────────────────────────────────────────────────── */
void    monitor_set_fixed_channel(uint8_t ch);
void    monitor_set_hopping(void);
uint8_t monitor_get_channel(void);

/* ── Frame-type filter ─────────────────────────────────────────────────── */
void monitor_set_filter(monitor_filter_t filter);

/* ── RSSI threshold ────────────────────────────────────────────────────── */
void monitor_set_rssi_threshold(int8_t min_rssi);

/* ── MAC address filter ────────────────────────────────────────────────── */
void monitor_set_mac_filter_mode(mac_filter_mode_t mode);
int  monitor_add_mac_filter(const uint8_t *mac);
int  monitor_remove_mac_filter(const uint8_t *mac);
void monitor_clear_mac_filter(void);

/* ── Statistics ────────────────────────────────────────────────────────── */
void monitor_get_stats(uint32_t *captured, uint32_t *dropped_ring,
                       uint32_t *dropped_pool, uint32_t *peak_pool);
void monitor_reset_stats(void);

#endif /* MONITOR_H */
