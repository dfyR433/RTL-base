/**
 * @file monitor.c
 * @brief Wi-Fi monitor-mode capture engine — packet-stream UART output.
 *
 * Output format MONITOR_FORMAT:
 *
 *   MONITOR_FMT_CAP    – libpcap, raw IEEE 802.11 (linktype 105)
 *   MONITOR_FMT_PCAP   – libpcap, 802.11 + radiotap (linktype 127)
 *   MONITOR_FMT_PCAPNG – pcapng,  802.11 + radiotap (linktype 127) [default]
 */

#include "ameba_soc.h"
#include "wifi_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "monitor.h"

extern serial_t monitor_uart;

/* =========================================================================
 * Little-endian write helpers
 * ========================================================================= */
static inline void write_u16_le(uint8_t *b, uint16_t v) {
    b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8);
}
static inline void write_u32_le(uint8_t *b, uint32_t v) {
    b[0] = (uint8_t)v;         b[1] = (uint8_t)(v >> 8);
    b[2] = (uint8_t)(v >> 16); b[3] = (uint8_t)(v >> 24);
}
static inline void write_u64_le(uint8_t *b, uint64_t v) {
    write_u32_le(b,     (uint32_t)v);
    write_u32_le(b + 4, (uint32_t)(v >> 32));
}

/* =========================================================================
 * UART — packet stream
 * ========================================================================= */
static void uart_write_raw(const void *data, size_t len) {
    const uint8_t *p = (const uint8_t *)data;
    for (size_t i = 0; i < len; i++) {
        while (!serial_writable(&monitor_uart)) { }
        serial_putc(&monitor_uart, p[i]);
    }
}

/* =========================================================================
 * Packet pool — lock-free allocation via atomics
 * ========================================================================= */
typedef struct {
    uint8_t  buffer[PACKET_BUFFER_SIZE];
    uint32_t in_use;
} pool_buf_t;

static pool_buf_t packet_pool[PACKET_POOL_SIZE];

volatile uint32_t stats_captured       = 0;
volatile uint32_t stats_dropped_ring   = 0;
volatile uint32_t stats_dropped_pool   = 0;
volatile uint32_t stats_peak_pool_used = 0;

static uint8_t *pool_alloc(void) {
    for (int i = 0; i < PACKET_POOL_SIZE; i++) {
        uint32_t expected = 0;
        if (__atomic_compare_exchange_n(&packet_pool[i].in_use, &expected, 1,
                                        false,
                                        __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
            uint32_t used = 0;
            for (int j = 0; j < PACKET_POOL_SIZE; j++)
                if (__atomic_load_n(&packet_pool[j].in_use, __ATOMIC_RELAXED))
                    used++;

            uint32_t old = __atomic_load_n(&stats_peak_pool_used, __ATOMIC_RELAXED);
            while (used > old)
                if (__atomic_compare_exchange_weak_n(&stats_peak_pool_used, &old, used,
                                                    __ATOMIC_RELAXED, __ATOMIC_RELAXED))
                    break;

            return packet_pool[i].buffer;
        }
    }
    return NULL;
}

static void pool_free(uint8_t *buf) {
    if (!buf) return;
    for (int i = 0; i < PACKET_POOL_SIZE; i++) {
        if (packet_pool[i].buffer == buf) {
            __atomic_store_n(&packet_pool[i].in_use, 0, __ATOMIC_RELEASE);
            return;
        }
    }
}

/* =========================================================================
 * Lock-free SPSC ring buffer
 * ========================================================================= */
typedef struct {
    uint8_t *data;
    uint32_t orig_len;
    uint32_t len;
    uint8_t  channel;
    int8_t   rssi;
    uint8_t  rate;
    uint64_t tsf_us;
} rb_entry_t;

static rb_entry_t    ring[RING_SIZE];
static volatile uint32_t write_idx = 0;
static volatile uint32_t read_idx  = 0;

static inline uint32_t ring_next(uint32_t i) { return (i + 1) % RING_SIZE; }

/* =========================================================================
 * Task handles and writer queue
 * ========================================================================= */
#define WRITER_QUEUE_LENGTH  16

static QueueHandle_t writer_queue        = NULL;
static TaskHandle_t  writer_task_handle  = NULL;
static TaskHandle_t  monitor_task_handle = NULL;
static TaskHandle_t  hopper_task_handle  = NULL;

/* =========================================================================
 * Writer task
 * ========================================================================= */
static void writer_task(void *param) {
    (void)param;
    uint8_t *block;
    while (1) {
        if (xQueueReceive(writer_queue, &block, portMAX_DELAY) == pdTRUE) {
            uint32_t blen = *(uint32_t *)(block + 4);
            uart_write_raw(block, blen);
            vPortFree(block);
        }
    }
}

/* =========================================================================
 * Hopper task
 * ========================================================================= */
static volatile uint8_t fixed_channel   = 0;
static volatile uint8_t current_channel = 0;

static void hopper_task(void *arg) {
    (void)arg;
    size_t idx = 0;
    while (1) {
        uint8_t ch = (fixed_channel != 0) ? fixed_channel : CHANNEL_LIST[idx];
        if (ch != current_channel) {
            wifi_set_channel(WLAN_IDX, ch);
            current_channel = ch;
        }
        if (fixed_channel == 0)
            idx = (idx + 1) % CHANNEL_LIST_LEN;
        vTaskDelay(pdMS_TO_TICKS(HOP_INTERVAL_MS));
    }
}

/* =========================================================================
 * Frame-type filter
 * ========================================================================= */
static monitor_filter_t current_filter = FILTER_ALL;

void monitor_set_filter(monitor_filter_t filter) { current_filter = filter; }

static bool should_capture(const uint8_t *buf, uint32_t len) {
    if (current_filter == FILTER_ALL) return true;
    if (len < 2) return false;

    uint16_t fc      = (uint16_t)(buf[0] | (buf[1] << 8));
    uint8_t  type    = (fc >> 2) & 0x3;
    uint8_t  subtype = (fc >> 4) & 0xF;

    switch (current_filter) {
        case FILTER_DATA:       return (type == 2);
        case FILTER_MANAGEMENT: return (type == 0);
        case FILTER_CONTROL:    return (type == 1);
        case FILTER_BEACON:     return (type == 0 && subtype == 8);
        case FILTER_PROBE_REQ:  return (type == 0 && subtype == 4);
        case FILTER_PROBE_RSP:  return (type == 0 && subtype == 5);
        default:                return true;
    }
}

/* =========================================================================
 * RSSI threshold filter
 * ========================================================================= */
static volatile int8_t rssi_threshold = -128;

void monitor_set_rssi_threshold(int8_t min_rssi) { rssi_threshold = min_rssi; }

/* =========================================================================
 * MAC address filter
 * ========================================================================= */
static mac_filter_mode_t mac_filter_mode = MAC_FILTER_NONE;
#define MAX_MAC_ENTRIES 16
static uint8_t mac_filter_list[MAX_MAC_ENTRIES][6];
static int     mac_filter_count = 0;

void monitor_set_mac_filter_mode(mac_filter_mode_t m) { mac_filter_mode = m; }

int monitor_add_mac_filter(const uint8_t *mac) {
    if (mac_filter_count >= MAX_MAC_ENTRIES) return -1;
    memcpy(mac_filter_list[mac_filter_count++], mac, 6);
    return 0;
}

int monitor_remove_mac_filter(const uint8_t *mac) {
    for (int i = 0; i < mac_filter_count; i++) {
        if (memcmp(mac_filter_list[i], mac, 6) == 0) {
            memmove(&mac_filter_list[i], &mac_filter_list[i + 1],
                    (mac_filter_count - i - 1) * 6);
            mac_filter_count--;
            return 0;
        }
    }
    return -1;
}

void monitor_clear_mac_filter(void) { mac_filter_count = 0; }

static bool check_mac_filter(const uint8_t *a1,
                              const uint8_t *a2,
                              const uint8_t *a3) {
    if (mac_filter_mode == MAC_FILTER_NONE) return true;
    for (int i = 0; i < mac_filter_count; i++) {
        const uint8_t *f = mac_filter_list[i];
        if (memcmp(a1,f,6)==0 || memcmp(a2,f,6)==0 || memcmp(a3,f,6)==0)
            return (mac_filter_mode == MAC_FILTER_ALLOWLIST);
    }
    return (mac_filter_mode == MAC_FILTER_DENYLIST);
}

/* =========================================================================
 * Promiscuous callback
 * ========================================================================= */
static u8 promisc_callback(struct rtw_rx_pkt_info *pkt_info) {
    if (!pkt_info || !pkt_info->buf || pkt_info->len == 0)
        return RTW_PROMISC_NEED_DRV_HDL;

    if (!should_capture(pkt_info->buf, pkt_info->len))
        return RTW_PROMISC_NEED_DRV_HDL;

    if ((int8_t)pkt_info->recv_signal_power < rssi_threshold)
        return RTW_PROMISC_NEED_DRV_HDL;

    if (mac_filter_mode != MAC_FILTER_NONE && pkt_info->len >= 24) {
        if (!check_mac_filter(pkt_info->buf + 4,
                              pkt_info->buf + 10,
                              pkt_info->buf + 16))
            return RTW_PROMISC_NEED_DRV_HDL;
    }

    uint32_t w    = __atomic_load_n(&write_idx, __ATOMIC_RELAXED);
    uint32_t r    = __atomic_load_n(&read_idx,  __ATOMIC_RELAXED);
    uint32_t next = ring_next(w);

    if (next == r) {
        __atomic_add_fetch(&stats_dropped_ring, 1, __ATOMIC_RELAXED);
        return RTW_PROMISC_NEED_DRV_HDL;
    }

    uint8_t *buf = pool_alloc();
    if (!buf) {
        __atomic_add_fetch(&stats_dropped_pool, 1, __ATOMIC_RELAXED);
        return RTW_PROMISC_NEED_DRV_HDL;
    }

    uint32_t copy_len = pkt_info->len;
    if (copy_len > PACKET_BUFFER_SIZE) copy_len = PACKET_BUFFER_SIZE;
    memcpy(buf, pkt_info->buf, copy_len);

    rb_entry_t *e = &ring[next];
    e->data     = buf;
    e->orig_len = pkt_info->len;
    e->len      = copy_len;
    e->channel  = (uint8_t)pkt_info->channel;
    e->rssi     = (int8_t)pkt_info->recv_signal_power;
    e->rate     = (uint8_t)pkt_info->data_rate;
    e->tsf_us   = wifi_get_tsf(WLAN_IDX);

    __atomic_store_n(&write_idx, next, __ATOMIC_RELEASE);
    __atomic_add_fetch(&stats_captured, 1, __ATOMIC_RELAXED);
    return RTW_PROMISC_NEED_DRV_HDL;
}

/* =========================================================================
 * Radiotap header
 * ========================================================================= */
#if (MONITOR_FORMAT == MONITOR_FMT_PCAP) || (MONITOR_FORMAT == MONITOR_FMT_PCAPNG)

static uint16_t ch_to_freq(uint8_t ch) {
    if (ch >= 1 && ch <= 13) return (uint16_t)(2407 + 5 * ch);
    if (ch == 14)             return 2484;
    return (uint16_t)(5000 + 5 * ch);
}

static size_t build_radiotap(uint8_t *out, size_t out_max,
                             uint8_t ch, int8_t rssi,
                             uint8_t rate_raw, uint64_t tsf_us) {
    static const uint32_t PRESENT =
        (1u << 0) | (1u << 2) | (1u << 3) | (1u << 5);

    if (out_max < 8) return 0;

    out[0] = 0; out[1] = 0;
    out[2] = 0; out[3] = 0;
    write_u32_le(&out[4], PRESENT);
    size_t off = 8;

    write_u64_le(&out[off], tsf_us); off += 8;

    unsigned rv = (unsigned)rate_raw * 2u;
    out[off++] = (uint8_t)(rv > 255u ? 255u : rv);

    if (off & 1) out[off++] = 0;
    write_u16_le(&out[off], ch_to_freq(ch));
    write_u16_le(&out[off + 2], (ch <= 14) ? 0x0080u : 0x0100u);
    off += 4;

    out[off++] = (uint8_t)rssi;

    size_t padded = (off + 3u) & ~3u;
    while (off < padded) out[off++] = 0;

    write_u16_le(&out[2], (uint16_t)off);
    return off;
}

#endif

/* =========================================================================
 * Helper
 * ========================================================================= */
static void queue_or_free(uint8_t *block) {
    if (writer_queue && xQueueSend(writer_queue, &block, 0) == pdTRUE)
        return;
    vPortFree(block);
    __atomic_add_fetch(&stats_dropped_ring, 1, __ATOMIC_RELAXED);
}

/* =========================================================================
 * FORMAT: MONITOR_FMT_CAP / MONITOR_FMT_PCAP
 * ========================================================================= */
#if (MONITOR_FORMAT == MONITOR_FMT_CAP) || (MONITOR_FORMAT == MONITOR_FMT_PCAP)

#define PCAP_ENV_HDR  8
static void write_pcap_global_header(void) {
    uint8_t h[24] = {0};
    write_u32_le(&h[0],  PCAP_MAGIC_US);
    write_u16_le(&h[4],  2);
    write_u16_le(&h[6],  4);
    write_u32_le(&h[16], PACKET_BUFFER_SIZE);
#if MONITOR_FORMAT == MONITOR_FMT_CAP
    write_u32_le(&h[20], LINKTYPE_IEEE802_11);
#else
    write_u32_le(&h[20], LINKTYPE_IEEE802_11_RADIOTAP);
#endif
    uart_write_raw(h, sizeof(h));
}

static void pcap_queue_packet(rb_entry_t *entry) {
#if MONITOR_FORMAT == MONITOR_FMT_PCAP
    uint8_t rt_buf[64];
    size_t  rt_len = build_radiotap(rt_buf, sizeof(rt_buf),
                                    entry->channel, entry->rssi,
                                    entry->rate, entry->tsf_us);
    if (rt_len == 0) return;
#else
    const uint8_t *rt_buf = NULL;
    size_t         rt_len = 0;
#endif

    uint32_t incl_len  = (uint32_t)(rt_len + entry->len);
    uint32_t orig_len  = (uint32_t)(rt_len + entry->orig_len);
    uint32_t rec_bytes = 16u + incl_len;
    uint32_t total     = (uint32_t)(PCAP_ENV_HDR + rec_bytes);

    uint8_t *block = (uint8_t *)pvPortMalloc(total);
    if (!block) return;

    write_u32_le(block + 0, 0);
    write_u32_le(block + 4, total);

    uint8_t *rec = block + PCAP_ENV_HDR;
    write_u32_le(rec +  0, (uint32_t)(entry->tsf_us / 1000000ULL));
    write_u32_le(rec +  4, (uint32_t)(entry->tsf_us % 1000000ULL));
    write_u32_le(rec +  8, incl_len);
    write_u32_le(rec + 12, orig_len);

    uint8_t *payload = rec + 16;
#if MONITOR_FORMAT == MONITOR_FMT_PCAP
    memcpy(payload, rt_buf, rt_len);
    payload += rt_len;
#endif
    memcpy(payload, entry->data, entry->len);

    queue_or_free(block);
}

static void pcap_writer_send(uint8_t *block) {
    uint32_t total = *(uint32_t *)(block + 4);
    uart_write_raw(block + PCAP_ENV_HDR,
                   total - PCAP_ENV_HDR);
    vPortFree(block);
}

#define write_session_header()   write_pcap_global_header()
#define write_packet_record(e)   pcap_queue_packet(e)
#define writer_send(b)           pcap_writer_send(b)

#endif

/* =========================================================================
 * FORMAT: MONITOR_FMT_PCAPNG
 * ========================================================================= */
#if MONITOR_FORMAT == MONITOR_FMT_PCAPNG

static void write_pcapng_shb(void) {
    uint8_t b[32] = {0};
    write_u32_le(&b[0],  0x0A0D0D0Au);
    write_u32_le(&b[4],  sizeof(b));
    write_u32_le(&b[8],  0x1A2B3C4Du);
    write_u16_le(&b[12], 1);
    write_u16_le(&b[14], 0);
    write_u64_le(&b[16], 0xFFFFFFFFFFFFFFFFULL);
    write_u32_le(&b[28], sizeof(b));
    uart_write_raw(b, sizeof(b));
}

static void write_pcapng_idb(void) {
    uint8_t b[64] = {0};
    size_t  p = 0;

    write_u32_le(&b[p], 0x00000001u); p += 4;
    write_u32_le(&b[p], 0);           p += 4;
    write_u16_le(&b[p], LINKTYPE_IEEE802_11_RADIOTAP); p += 2;
    write_u16_le(&b[p], 0);           p += 2;
    write_u32_le(&b[p], PACKET_BUFFER_SIZE); p += 4;

    write_u16_le(&b[p], IDB_OPT_TSRESOL); p += 2;
    write_u16_le(&b[p], 1);               p += 2;
    b[p++] = IDB_TSRESOL_NS; p += 3;

    write_u16_le(&b[p], IDB_OPT_FCSLEN);  p += 2;
    write_u16_le(&b[p], 1);               p += 2;
    b[p++] = IDB_FCS_LEN_BYTES; p += 3;

    write_u16_le(&b[p], 0); p += 2;
    write_u16_le(&b[p], 0); p += 2;

    uint32_t total = (uint32_t)(p + 4);
    write_u32_le(&b[4], total);
    write_u32_le(&b[p], total); p += 4;

    uart_write_raw(b, total);
}

static void pcapng_queue_epb(rb_entry_t *entry) {
    uint8_t rt_buf[64];
    size_t  rt_len = build_radiotap(rt_buf, sizeof(rt_buf),
                                    entry->channel, entry->rssi,
                                    entry->rate, entry->tsf_us);
    if (rt_len == 0) return;

    uint64_t ts_ns         = entry->tsf_us * 1000ULL;
    uint32_t caplen        = (uint32_t)(rt_len + entry->len);
    uint32_t padded_caplen = (caplen + 3u) & ~3u;
    uint32_t block_total   = 28u + padded_caplen + 4u;

    uint8_t *block = (uint8_t *)pvPortMalloc(block_total);
    if (!block) return;

    write_u32_le(block + 0,  0x00000006u);
    write_u32_le(block + 4,  block_total);
    write_u32_le(block + 8,  0);
    write_u32_le(block + 12, (uint32_t)(ts_ns >> 32));
    write_u32_le(block + 16, (uint32_t)ts_ns);
    write_u32_le(block + 20, caplen);
    write_u32_le(block + 24, entry->orig_len);

    memcpy(block + 28, rt_buf, rt_len);
    memcpy(block + 28 + rt_len, entry->data, entry->len);
    if (padded_caplen > caplen)
        memset(block + 28 + caplen, 0, padded_caplen - caplen);
    write_u32_le(block + 28 + padded_caplen, block_total);

    queue_or_free(block);
}

static void pcapng_writer_send(uint8_t *block) {
    uint32_t total = *(uint32_t *)(block + 4);
    uart_write_raw(block, total);
    vPortFree(block);
}

#define write_session_header()   do { write_pcapng_shb(); write_pcapng_idb(); } while(0)
#define write_packet_record(e)   pcapng_queue_epb(e)
#define writer_send(b)           pcapng_writer_send(b)

#endif

/* =========================================================================
 * Statistics
 * ========================================================================= */
void monitor_get_stats(uint32_t *cap, uint32_t *dr, uint32_t *dp, uint32_t *pk) {
    *cap = __atomic_load_n(&stats_captured,       __ATOMIC_RELAXED);
    *dr  = __atomic_load_n(&stats_dropped_ring,   __ATOMIC_RELAXED);
    *dp  = __atomic_load_n(&stats_dropped_pool,   __ATOMIC_RELAXED);
    *pk  = __atomic_load_n(&stats_peak_pool_used, __ATOMIC_RELAXED);
}

void monitor_reset_stats(void) {
    __atomic_store_n(&stats_captured,       0, __ATOMIC_RELAXED);
    __atomic_store_n(&stats_dropped_ring,   0, __ATOMIC_RELAXED);
    __atomic_store_n(&stats_dropped_pool,   0, __ATOMIC_RELAXED);
    __atomic_store_n(&stats_peak_pool_used, 0, __ATOMIC_RELAXED);
}

/* =========================================================================
 * Writer task (format-aware via writer_send macro)
 * ========================================================================= */
static void writer_task(void *param) {
    (void)param;
    uint8_t *block;
    while (1)
        if (xQueueReceive(writer_queue, &block, portMAX_DELAY) == pdTRUE)
            writer_send(block);
}

/* =========================================================================
 * Monitor task — drains ring, dispatches to format writer
 * ========================================================================= */
static int monitor_is_running = 0;

static void monitor_task(void *param) {
    (void)param;

    if (wifi_on(RTW_MODE_STA) != 0) { vTaskDelete(NULL); return; }

    for (int w = 0; w < 5000; w += 200) {
        if (wifi_is_running(WLAN_IDX)) break;
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    struct rtw_promisc_para para = {0};
    para.filter_mode = RTW_PROMISC_FILTER_ALL_PKT;
    para.callback    = promisc_callback;
    wifi_promisc_enable(1, &para);

    xTaskCreate(hopper_task, "hopper",
                HOP_TASK_STACK / sizeof(StackType_t),
                NULL, HOP_TASK_PRIO, &hopper_task_handle);

    write_session_header();

    monitor_task_handle = xTaskGetCurrentTaskHandle();

    while (1) {
        uint32_t r = __atomic_load_n(&read_idx,  __ATOMIC_RELAXED);
        uint32_t w = __atomic_load_n(&write_idx, __ATOMIC_ACQUIRE);

        while (r != w) {
            uint32_t next = ring_next(r);
            rb_entry_t entry;
            memcpy(&entry, &ring[next], sizeof(entry));
            __atomic_store_n(&read_idx, next, __ATOMIC_RELEASE);

            if (entry.data && entry.len > 0) {
                write_packet_record(&entry);
                pool_free(entry.data);
            }
            r = next;
            w = __atomic_load_n(&write_idx, __ATOMIC_ACQUIRE);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* =========================================================================
 * Public control
 * ========================================================================= */
void monitor_start(void) {
    if (monitor_is_running) return;

    writer_queue = xQueueCreate(WRITER_QUEUE_LENGTH, sizeof(uint8_t *));

    xTaskCreate(writer_task,  "writer",
                WRITER_TASK_STACK  / sizeof(StackType_t),
                NULL, WRITER_TASK_PRIO,  &writer_task_handle);
    xTaskCreate(monitor_task, "monitor",
                MONITOR_TASK_STACK / sizeof(StackType_t),
                NULL, MONITOR_TASK_PRIO, &monitor_task_handle);

    monitor_is_running = 1;
}

void monitor_stop(void) {
    if (!monitor_is_running) return;

    wifi_promisc_enable(0, NULL);

    if (monitor_task_handle) { vTaskDelete(monitor_task_handle); monitor_task_handle = NULL; }
    if (hopper_task_handle)  { vTaskDelete(hopper_task_handle);  hopper_task_handle  = NULL; }

    if (writer_queue) {
        uint8_t *stale;
        while (xQueueReceive(writer_queue, &stale, 0) == pdTRUE)
            if (stale) vPortFree(stale);
    }

    if (writer_task_handle)  { vTaskDelete(writer_task_handle);  writer_task_handle  = NULL; }
    if (writer_queue)        { vQueueDelete(writer_queue);       writer_queue        = NULL; }

    monitor_is_running = 0;
}

void monitor_set_fixed_channel(uint8_t ch) { fixed_channel = ch; }
void monitor_set_hopping(void)             { fixed_channel = 0;  }
uint8_t monitor_get_channel(void)          { return current_channel; }
