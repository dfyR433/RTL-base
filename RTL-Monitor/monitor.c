/*
 * Ameba Wi‑Fi Monitor – pcapng over UART (dual‑band hopping)
 *
 * Emits a valid pcapng stream:
 *   - Section Header Block (SHB)
 *   - Interface Description Block (IDB) with linktype 127 (radiotap)
 *   - Enhanced Packet Blocks (EPB) containing radiotap + 802.11 frame
 */

#include "ameba_soc.h"
#include "wifi_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "serial_api.h"
#include "monitor.h"

/* -------------------------------------------------------------------------
 * Little‑endian write helpers
 * ------------------------------------------------------------------------- */
static inline void write_u16_le(uint8_t *buf, uint16_t val) {
    buf[0] = (uint8_t)val;
    buf[1] = (uint8_t)(val >> 8);
}

static inline void write_u32_le(uint8_t *buf, uint32_t val) {
    buf[0] = (uint8_t)val;
    buf[1] = (uint8_t)(val >> 8);
    buf[2] = (uint8_t)(val >> 16);
    buf[3] = (uint8_t)(val >> 24);
}

static inline void write_u64_le(uint8_t *buf, uint64_t val) {
    write_u32_le(buf, (uint32_t)val);
    write_u32_le(buf + 4, (uint32_t)(val >> 32));
}

/* -------------------------------------------------------------------------
 * Packet pool – fixed‑size buffers (ISR‑safe using atomic ops)
 * ------------------------------------------------------------------------- */
typedef struct {
    uint8_t  buffer[PACKET_BUFFER_SIZE];
    uint32_t in_use;
} pool_buf_t;

static pool_buf_t packet_pool[PACKET_POOL_SIZE];

static uint8_t* pool_alloc(void) {
    for (int i = 0; i < PACKET_POOL_SIZE; i++) {
        uint32_t expected = 0;
        if (__atomic_compare_exchange_n(&packet_pool[i].in_use,
                                        &expected, 1,
                                        false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
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

/* -------------------------------------------------------------------------
 * Ring buffer for packet metadata
 * ------------------------------------------------------------------------- */
typedef struct {
    uint8_t *data;
    uint32_t orig_len;
    uint32_t len;
    uint8_t  channel;
    int8_t   rssi;
    uint8_t  rate;
} rb_entry_t;

static rb_entry_t ring[RING_SIZE];
static volatile uint32_t write_idx = 0;
static volatile uint32_t read_idx  = 0;

static inline uint32_t ring_next(uint32_t i) { return (i + 1) % RING_SIZE; }

/* -------------------------------------------------------------------------
 * Diagnostics counters (atomic)
 * ------------------------------------------------------------------------- */
static volatile uint32_t stats_captured = 0;
static volatile uint32_t stats_dropped_ring = 0;
static volatile uint32_t stats_dropped_pool = 0;

/* -------------------------------------------------------------------------
 * High‑resolution timestamp (nanoseconds)
 * ------------------------------------------------------------------------- */
#ifdef USE_DWT_CYCCNT
/* CMSIS‑style DWT access – CoreDebug and DWT structures are defined in
 * core_cm33.h (included via ameba_soc.h) */
static void dwt_init_if_available(void) {
    /* Enable DWT cycle counter */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
#endif /* USE_DWT_CYCCNT */

static uint64_t get_timestamp_ns(void) {
#ifdef USE_DWT_CYCCNT
    static uint32_t prev = 0;
    static uint64_t wrap = 0;
    uint32_t cur = DWT->CYCCNT;
    if (cur < prev) {
        wrap += (1ULL << 32);
    }
    prev = cur;
    uint64_t cycles64 = wrap + (uint64_t)cur;
#if defined(__SIZEOF_INT128__)
    return (uint64_t)((__uint128_t)cycles64 * 1000000000ULL / (uint64_t)SystemCoreClock);
#else
    uint64_t cpu_hz = (uint64_t)SystemCoreClock;
    uint64_t seconds = cycles64 / cpu_hz;
    uint64_t rem = cycles64 % cpu_hz;
    return seconds * 1000000000ULL + (rem * 1000000000ULL) / cpu_hz;
#endif
#elif defined(PLATFORM_TIMER_US)
    uint64_t us = platform_get_time_us();
    return us * 1000ULL;
#else
    TickType_t ticks = xTaskGetTickCount();
    uint64_t ms = (uint64_t)ticks * portTICK_PERIOD_MS;
    return ms * 1000000ULL;
#endif
}

/* -------------------------------------------------------------------------
 * UART output – raw binary
 * ------------------------------------------------------------------------- */
static serial_t uart;

static void uart_write_raw(const void *data, size_t len) {
    const uint8_t *p = (const uint8_t*)data;
    for (size_t i = 0; i < len; i++) {
        /* Busy‑wait until UART is ready to transmit – no voluntary yield */
        while (!serial_writable(&uart)) { }
        serial_putc(&uart, p[i]);
    }
}

/* -------------------------------------------------------------------------
 * Channel hopping task
 * ------------------------------------------------------------------------- */
static void hopper_task(void *arg) {
    (void)arg;
    size_t idx = 0;
    while (1) {
        uint8_t ch = CHANNEL_LIST[idx];
        wifi_set_channel(WLAN_IDX, ch);
        idx = (idx + 1) % CHANNEL_LIST_LEN;
        vTaskDelay(pdMS_TO_TICKS(HOP_INTERVAL_MS));
    }
}

/* -------------------------------------------------------------------------
 * Promiscuous callback – runs in interrupt/softirq context
 * ------------------------------------------------------------------------- */
static u8 promisc_callback(struct rtw_rx_pkt_info *pkt_info) {
    if (!pkt_info || !pkt_info->buf || pkt_info->len == 0)
        return RTW_PROMISC_NEED_DRV_HDL;

    uint32_t w = __atomic_load_n(&write_idx, __ATOMIC_RELAXED);
    uint32_t r = __atomic_load_n(&read_idx, __ATOMIC_RELAXED);
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
    if (copy_len > PACKET_BUFFER_SIZE)
        copy_len = PACKET_BUFFER_SIZE;
    memcpy(buf, pkt_info->buf, copy_len);

    rb_entry_t *e = &ring[next];
    e->data     = buf;
    e->orig_len = pkt_info->len;
    e->len      = copy_len;
    e->channel  = (uint8_t)pkt_info->channel;
    e->rssi     = (int8_t)pkt_info->recv_signal_power;
    e->rate     = (uint8_t)pkt_info->data_rate;

    __atomic_store_n(&write_idx, next, __ATOMIC_RELEASE);
    __atomic_add_fetch(&stats_captured, 1, __ATOMIC_RELAXED);

    return RTW_PROMISC_NEED_DRV_HDL;
}

/* -------------------------------------------------------------------------
 * Helper: channel to frequency (MHz)
 * ------------------------------------------------------------------------- */
static uint16_t channel_to_freq_mhz(uint8_t ch) {
    if (ch >= 1 && ch <= 13)
        return (uint16_t)(2407 + 5 * ch);
    else if (ch == 14)
        return 2484;
    else
        return (uint16_t)(5000 + 5 * ch);
}

/* -------------------------------------------------------------------------
 * Convert data rate from SDK into radiotap rate (units of 500 kb/s)
 * ------------------------------------------------------------------------- */
static uint8_t convert_rate_to_radiotap(uint8_t sdk_rate_mbps) {
    if (sdk_rate_mbps == 0) return 0;
    unsigned int v = (unsigned int)sdk_rate_mbps * 2u;
    if (v > 255) v = 255;
    return (uint8_t)v;
}

/* -------------------------------------------------------------------------
 * Build radiotap header
 * ------------------------------------------------------------------------- */
static size_t build_radiotap(uint8_t *out, size_t out_max,
                             uint8_t channel, int8_t rssi, uint8_t rate) {
    const uint32_t present = (1u << 2) | (1u << 3) | (1u << 5);
    size_t off = 0;

    if (out_max < 8) return 0;
    out[off++] = 0;          /* version */
    out[off++] = 0;          /* pad */
    off += 2;                /* it_len placeholder */
    write_u32_le(&out[4], present);
    off = 8;

    /* Rate field */
    out[off++] = rate;

    /* Channel field */
    if ((off & 1) != 0) out[off++] = 0;
    uint16_t freq = channel_to_freq_mhz(channel);
    write_u16_le(&out[off], freq);
    uint16_t flags = (channel <= 14) ? 0x0080 : 0x0100;   /* 2.4 GHz / 5 GHz */
    write_u16_le(&out[off+2], flags);
    off += 4;

    /* Antenna signal */
    out[off++] = (uint8_t)rssi;

    /* Pad to 32‑bit boundary */
    size_t padded = (off + 3) & ~3;
    while (off < padded) out[off++] = 0;

    write_u16_le(&out[2], (uint16_t)off);
    return off;
}

/* -------------------------------------------------------------------------
 * Write Section Header Block (SHB)
 * ------------------------------------------------------------------------- */
static void write_pcapng_shb(void) {
    uint8_t block[40] = {0};
    write_u32_le(&block[0],  0x0A0D0D0A);
    write_u32_le(&block[4],  sizeof(block));
    write_u32_le(&block[8],  0x1A2B3C4D);
    write_u16_le(&block[12], 1);
    write_u16_le(&block[14], 0);
    write_u64_le(&block[16], 0xFFFFFFFFFFFFFFFFULL);
    /* Options: end-of-options */
    write_u16_le(&block[32], 0);
    write_u16_le(&block[34], 0);
    write_u32_le(&block[36], sizeof(block));
    uart_write_raw(block, sizeof(block));
}

/* -------------------------------------------------------------------------
 * Write Interface Description Block (IDB)
 * ------------------------------------------------------------------------- */
static void write_pcapng_idb(void) {
    uint8_t block[64] = {0};
    size_t pos = 0;

    write_u32_le(&block[pos], 0x00000001); pos += 4;
    write_u32_le(&block[pos], 0);          pos += 4;    /* total len placeholder */

    write_u16_le(&block[pos], 127); pos += 2;           /* LINKTYPE_IEEE802_11_RADIOTAP */
    write_u16_le(&block[pos], 0);   pos += 2;
    write_u32_le(&block[pos], PACKET_BUFFER_SIZE); pos += 4;

    /* if_tsresol */
    write_u16_le(&block[pos], 9);  pos += 2;
    write_u16_le(&block[pos], 1);  pos += 2;
    block[pos++] = IDB_TSRESOL;
    pos += 3;

    /* if_fcslen */
    write_u16_le(&block[pos], 11); pos += 2;
    write_u16_le(&block[pos], 1);  pos += 2;
    block[pos++] = IDB_FCS_LEN;
    pos += 3;

    /* end-of-options */
    write_u16_le(&block[pos], 0); pos += 2;
    write_u16_le(&block[pos], 0); pos += 2;

    uint32_t total_len = pos + 4;
    write_u32_le(&block[4], total_len);
    write_u32_le(&block[pos], total_len); pos += 4;

    uart_write_raw(block, total_len);
}

/* -------------------------------------------------------------------------
 * Write one Enhanced Packet Block (EPB)
 * ------------------------------------------------------------------------- */
static void write_pcapng_epb(rb_entry_t *entry) {
    uint8_t radiotap_buf[64];
    uint8_t rt_rate = convert_rate_to_radiotap(entry->rate);
    size_t radiotap_len = build_radiotap(radiotap_buf, sizeof(radiotap_buf),
                                         entry->channel, entry->rssi, rt_rate);
    if (radiotap_len == 0) return;

    uint32_t caplen = (uint32_t)radiotap_len + entry->len;
    uint32_t padded_caplen = (caplen + 3) & ~3;
    uint32_t block_total = 28 + padded_caplen + 4;

    uint8_t *block = (uint8_t*)pvPortMalloc(block_total);
    if (!block) return;

    write_u32_le(block + 0,  0x00000006);
    write_u32_le(block + 4,  block_total);
    write_u32_le(block + 8,  0);                     /* Interface ID = 0 */

    uint64_t ts = get_timestamp_ns();
    write_u32_le(block + 12, (uint32_t)(ts >> 32));
    write_u32_le(block + 16, (uint32_t)ts);
    write_u32_le(block + 20, caplen);
    write_u32_le(block + 24, entry->orig_len);

    memcpy(block + 28, radiotap_buf, radiotap_len);
    memcpy(block + 28 + radiotap_len, entry->data, entry->len);
    if (padded_caplen > caplen) {
        memset(block + 28 + caplen, 0, padded_caplen - caplen);
    }

    write_u32_le(block + 28 + padded_caplen, block_total);

    uart_write_raw(block, block_total);
    vPortFree(block);
}

/* -------------------------------------------------------------------------
 * Monitor main task
 * ------------------------------------------------------------------------- */
static void monitor_task(void *param) {
    (void)param;

    /* Initialise UART */
    serial_init(&uart, TX_PIN, RX_PIN);
    serial_baud(&uart, BAUD_RATE);
    serial_format(&uart, 8, ParityNone, 1);

    /* Start Wi‑Fi */
    if (wifi_on(RTW_MODE_STA) != 0) {
        /* No output – silently hang or restart */
        vTaskDelete(NULL);
        return;
    }

    int waited = 0;
    while (waited < 5000) {
        if (wifi_is_running(WLAN_IDX)) break;
        vTaskDelay(pdMS_TO_TICKS(200));
        waited += 200;
    }

#ifdef USE_DWT_CYCCNT
    /* Init DWT if available for high‑resolution timestamps */
    dwt_init_if_available();
#endif

    /* Enable promiscuous mode */
    struct rtw_promisc_para para;
    memset(&para, 0, sizeof(para));
    para.filter_mode = RTW_PROMISC_FILTER_ALL_PKT;
    para.callback = promisc_callback;
    wifi_promisc_enable(1, &para);

    /* Start channel hopper */
    xTaskCreate(hopper_task, "hopper", HOP_TASK_STACK / sizeof(StackType_t),
                NULL, HOP_TASK_PRIO, NULL);

    /* Emit pcapng global headers */
    write_pcapng_shb();
    write_pcapng_idb();

    /* Main loop: consume ring and emit EPBs – no delay */
    while (1) {
        uint32_t r = __atomic_load_n(&read_idx, __ATOMIC_RELAXED);
        uint32_t w = __atomic_load_n(&write_idx, __ATOMIC_ACQUIRE);

        while (r != w) {
            uint32_t next = ring_next(r);
            rb_entry_t entry;
            memcpy(&entry, &ring[next], sizeof(entry));
            __atomic_store_n(&read_idx, next, __ATOMIC_RELEASE);

            if (entry.data && entry.len > 0) {
                write_pcapng_epb(&entry);
                pool_free(entry.data);
            }

            r = next;
            w = __atomic_load_n(&write_idx, __ATOMIC_ACQUIRE);
        }
    }
}

/* -------------------------------------------------------------------------
 * Public interface
 * ------------------------------------------------------------------------- */
void monitor_start(void) {
    xTaskCreate(monitor_task, "monitor", MONITOR_TASK_STACK / sizeof(StackType_t),
                NULL, MONITOR_TASK_PRIO, NULL);
}
