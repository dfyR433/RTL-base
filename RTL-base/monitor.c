#include "ameba_soc.h"
#include "wifi_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "serial_api.h"
#include "monitor.h"

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

typedef struct {
    uint8_t  buffer[PACKET_BUFFER_SIZE];
    uint32_t in_use;
} pool_buf_t;

static pool_buf_t packet_pool[PACKET_POOL_SIZE];

volatile uint32_t stats_captured = 0;
volatile uint32_t stats_dropped_ring = 0;
volatile uint32_t stats_dropped_pool = 0;
volatile uint32_t stats_peak_pool_used = 0;

static uint8_t* pool_alloc(void) {
    for (int i = 0; i < PACKET_POOL_SIZE; i++) {
        uint32_t expected = 0;
        if (__atomic_compare_exchange_n(&packet_pool[i].in_use,
                                        &expected, 1,
                                        false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
            uint32_t used = 0;
            for (int j = 0; j < PACKET_POOL_SIZE; j++) {
                if (__atomic_load_n(&packet_pool[j].in_use, __ATOMIC_RELAXED)) used++;
            }
            if (used > stats_peak_pool_used) stats_peak_pool_used = used;
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

#ifdef USE_DWT_CYCCNT
static void dwt_init_if_available(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
#endif

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

serial_t uart;

static void uart_write_raw(const void *data, size_t len) {
    const uint8_t *p = (const uint8_t*)data;
    for (size_t i = 0; i < len; i++) {
        while (!serial_writable(&uart)) { }
        serial_putc(&uart, p[i]);
    }
}

#define WRITER_QUEUE_LENGTH    16
static QueueHandle_t writer_queue = NULL;
static TaskHandle_t writer_task_handle = NULL;
static TaskHandle_t monitor_task_handle = NULL;
static TaskHandle_t hopper_task_handle = NULL;
static volatile uint8_t fixed_channel = 0;
static int monitor_is_running = 0;

static void writer_task(void *param) {
    (void)param;
    uint8_t *block;
    while (1) {
        if (xQueueReceive(writer_queue, &block, portMAX_DELAY) == pdTRUE) {
            uint32_t block_len = *(uint32_t*)block;
            uart_write_raw(block, block_len);
            vPortFree(block);
        }
    }
}

static void hopper_task(void *arg) {
    (void)arg;
    size_t idx = 0;
    while (1) {
        if (fixed_channel != 0) {
            wifi_set_channel(WLAN_IDX, fixed_channel);
        } else {
            uint8_t ch = CHANNEL_LIST[idx];
            wifi_set_channel(WLAN_IDX, ch);
            idx = (idx + 1) % CHANNEL_LIST_LEN;
        }
        vTaskDelay(pdMS_TO_TICKS(HOP_INTERVAL_MS));
    }
}

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

static uint16_t channel_to_freq_mhz(uint8_t ch) {
    if (ch >= 1 && ch <= 13)
        return (uint16_t)(2407 + 5 * ch);
    else if (ch == 14)
        return 2484;
    else
        return (uint16_t)(5000 + 5 * ch);
}

static uint8_t convert_rate_to_radiotap(uint8_t sdk_rate_mbps) {
    if (sdk_rate_mbps == 0) return 0;
    unsigned int v = (unsigned int)sdk_rate_mbps * 2u;
    if (v > 255) v = 255;
    return (uint8_t)v;
}

static size_t build_radiotap(uint8_t *out, size_t out_max,
                             uint8_t channel, int8_t rssi, uint8_t rate) {
    const uint32_t present = (1u << 2) | (1u << 3) | (1u << 5);
    size_t off = 0;

    if (out_max < 8) return 0;
    out[off++] = 0;
    out[off++] = 0;
    off += 2;
    write_u32_le(&out[4], present);
    off = 8;

    out[off++] = rate;

    if ((off & 1) != 0) out[off++] = 0;
    uint16_t freq = channel_to_freq_mhz(channel);
    write_u16_le(&out[off], freq);
    uint16_t flags = (channel <= 14) ? 0x0080 : 0x0100;
    write_u16_le(&out[off+2], flags);
    off += 4;

    out[off++] = (uint8_t)rssi;

    size_t padded = (off + 3) & ~3;
    while (off < padded) out[off++] = 0;

    write_u16_le(&out[2], (uint16_t)off);
    return off;
}

static void write_pcapng_shb(void) {
    uint8_t block[40] = {0};
    write_u32_le(&block[0],  0x0A0D0D0A);
    write_u32_le(&block[4],  sizeof(block));
    write_u32_le(&block[8],  0x1A2B3C4D);
    write_u16_le(&block[12], 1);
    write_u16_le(&block[14], 0);
    write_u64_le(&block[16], 0xFFFFFFFFFFFFFFFFULL);
    write_u16_le(&block[32], 0);
    write_u16_le(&block[34], 0);
    write_u32_le(&block[36], sizeof(block));
    uart_write_raw(block, sizeof(block));
}

static void write_pcapng_idb(void) {
    uint8_t block[64] = {0};
    size_t pos = 0;

    write_u32_le(&block[pos], 0x00000001); pos += 4;
    write_u32_le(&block[pos], 0);          pos += 4;

    write_u16_le(&block[pos], 127); pos += 2;
    write_u16_le(&block[pos], 0);   pos += 2;
    write_u32_le(&block[pos], PACKET_BUFFER_SIZE); pos += 4;

    write_u16_le(&block[pos], 9);  pos += 2;
    write_u16_le(&block[pos], 1);  pos += 2;
    block[pos++] = IDB_TSRESOL;
    pos += 3;

    write_u16_le(&block[pos], 11); pos += 2;
    write_u16_le(&block[pos], 1);  pos += 2;
    block[pos++] = IDB_FCS_LEN;
    pos += 3;

    write_u16_le(&block[pos], 0); pos += 2;
    write_u16_le(&block[pos], 0); pos += 2;

    uint32_t total_len = pos + 4;
    write_u32_le(&block[4], total_len);
    write_u32_le(&block[pos], total_len); pos += 4;

    uart_write_raw(block, total_len);
}

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
    write_u32_le(block + 8,  0);

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

    if (writer_queue != NULL) {
        if (xQueueSend(writer_queue, &block, 0) != pdTRUE) {
            vPortFree(block);
            __atomic_add_fetch(&stats_dropped_ring, 1, __ATOMIC_RELAXED);
        }
    } else {
        uart_write_raw(block, block_total);
        vPortFree(block);
    }
}

static void monitor_task(void *param) {
    (void)param;

    serial_init(&uart, TX_PIN, RX_PIN);
    serial_baud(&uart, BAUD_RATE);
    serial_format(&uart, 8, ParityNone, 1);

    if (wifi_on(RTW_MODE_STA) != 0) {
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
    dwt_init_if_available();
#endif

    struct rtw_promisc_para para;
    memset(&para, 0, sizeof(para));
    para.filter_mode = RTW_PROMISC_FILTER_ALL_PKT;
    para.callback = promisc_callback;
    wifi_promisc_enable(1, &para);

    xTaskCreate(hopper_task, "hopper", HOP_TASK_STACK / sizeof(StackType_t),
                NULL, HOP_TASK_PRIO, &hopper_task_handle);

    write_pcapng_shb();
    write_pcapng_idb();

    monitor_task_handle = xTaskGetCurrentTaskHandle();

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

void monitor_start(void) {
    if (monitor_is_running) return;
    writer_queue = xQueueCreate(WRITER_QUEUE_LENGTH, sizeof(uint8_t*));
    xTaskCreate(writer_task, "writer", WRITER_TASK_STACK / sizeof(StackType_t),
                NULL, WRITER_TASK_PRIO, &writer_task_handle);
    xTaskCreate(monitor_task, "monitor", MONITOR_TASK_STACK / sizeof(StackType_t),
                NULL, MONITOR_TASK_PRIO, &monitor_task_handle);
    monitor_is_running = 1;
}

void monitor_stop(void) {
    if (!monitor_is_running) return;
    wifi_promisc_enable(0, NULL);
    if (monitor_task_handle) {
        vTaskDelete(monitor_task_handle);
        monitor_task_handle = NULL;
    }
    if (hopper_task_handle) {
        vTaskDelete(hopper_task_handle);
        hopper_task_handle = NULL;
    }
    if (writer_task_handle) {
        vTaskDelete(writer_task_handle);
        writer_task_handle = NULL;
    }
    if (writer_queue) {
        vQueueDelete(writer_queue);
        writer_queue = NULL;
    }
    monitor_is_running = 0;
}

void monitor_set_fixed_channel(uint8_t ch) {
    fixed_channel = ch;
}

void monitor_set_hopping(void) {
    fixed_channel = 0;
}

int monitor_read_char(void) {
    return serial_getc(&uart);
}
