/**
 * @file  monitor.c
 * @brief Wi-Fi monitor-mode capture — RTL8721Dx.
 */

#include "monitor.h"
#include "timer.h"
#include "ameba_soc.h"
#include "wifi_api.h"
#include "wifi_api_ext.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "serial_api.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

extern serial_t monitor_uart;

/* =========================================================================
 * Compile-time assertions
 * ========================================================================= */
_Static_assert((RING_SIZE & (RING_SIZE - 1u)) == 0u,
               "RING_SIZE must be a power of two");
_Static_assert(BLOCK_MAX_SIZE >= 32u + 32u + PACKET_BUFFER_SIZE,
               "BLOCK_MAX_SIZE too small for largest EPB");
_Static_assert(PACKET_POOL_SIZE >= 4u && PACKET_POOL_SIZE <= 256u,
               "PACKET_POOL_SIZE out of range (index type is uint8_t, max 255)");
_Static_assert(BLOCK_POOL_SIZE >= 2u,
               "BLOCK_POOL_SIZE must be at least 2");
_Static_assert(CHANNEL_LIST_LEN == 38u,
               "Update CHANNEL_LIST_LEN if channel list changes");

/* =========================================================================
 * Channel list
 * ========================================================================= */
static const uint8_t s_channel_list[CHANNEL_LIST_LEN] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
    36, 40, 44, 48, 52, 56, 60, 64,
    100,104,108,112,116,120,124,128,132,136,140,144,
    149,153,157,161,165
};
_Static_assert(sizeof(s_channel_list) == CHANNEL_LIST_LEN,
               "Channel list length mismatch");

/* =========================================================================
 * Per-channel statistics (Welford RSSI variance + traffic EMA).
 * ========================================================================= */
static channel_stats_t s_ch_stats[CHANNEL_LIST_LEN];

static int channel_to_idx(uint8_t ch) {
    for (int i = 0; i < (int)CHANNEL_LIST_LEN; i++)
        if (s_channel_list[i] == ch) return i;
    return -1;
}

static void ch_stat_update(uint8_t ch, int8_t rssi, uint8_t fc_type)
{
    int idx = channel_to_idx(ch);
    if (idx < 0) return;
    channel_stats_t *s = &s_ch_stats[idx];
    s->frames_total++;
    switch (fc_type) {
        case 0: s->frames_mgmt++;  break;
        case 1: s->frames_ctrl++;  break;
        case 2: s->frames_data++;  break;
        default:s->frames_other++; break;
    }
    if (s->frames_total == 1u) { s->rssi_min = rssi; s->rssi_max = rssi; }
    else {
        if (rssi < s->rssi_min) s->rssi_min = rssi;
        if (rssi > s->rssi_max) s->rssi_max = rssi;
    }
    s->rssi_sum += (int32_t)rssi;

    if (s->frames_total >= 2u) {
        int32_t cur_mean  = (int32_t)((s->rssi_sum << 8)
                            / (int32_t)s->frames_total);
        int32_t prev_mean = (int32_t)(((s->rssi_sum - (int32_t)rssi) << 8)
                            / (int32_t)(s->frames_total - 1u));
        int32_t delta_cur  = ((int32_t)rssi << 8) - cur_mean;
        int32_t delta_prev = ((int32_t)rssi << 8) - prev_mean;
        s->rssi_m2 += (int64_t)delta_cur * delta_prev;
    }

    static const int8_t thr[RSSI_HIST_BINS - 1u] = {-90,-80,-70,-60,-50,-40,-30};
    uint32_t bucket = RSSI_HIST_BINS - 1u;
    for (uint32_t b = 0; b < RSSI_HIST_BINS - 1u; b++) {
        if (rssi < thr[b]) { bucket = b; break; }
    }
    s->rssi_histogram[bucket]++;

    uint32_t ema = __atomic_load_n(&s->traffic_ema, __ATOMIC_RELAXED);
    __atomic_store_n(&s->traffic_ema, (ema * 7u + 256u) / 8u, __ATOMIC_RELAXED);
}

int monitor_get_channel_stats(uint8_t ch, channel_stats_t *out) {
    if (!out) return -1;
    int idx = channel_to_idx(ch);
    if (idx < 0) return -1;
    taskENTER_CRITICAL(); *out = s_ch_stats[idx]; taskEXIT_CRITICAL();
    return 0;
}
void monitor_reset_channel_stats(void) {
    taskENTER_CRITICAL();
    memset(s_ch_stats, 0, sizeof(s_ch_stats));
    taskEXIT_CRITICAL();
}

/* =========================================================================
 * LE wire-format helpers
 * ========================================================================= */
static inline void w16(uint8_t *b, uint16_t v)
{ b[0]=(uint8_t)v; b[1]=(uint8_t)(v>>8); }
static inline void w32(uint8_t *b, uint32_t v)
{ b[0]=(uint8_t)v; b[1]=(uint8_t)(v>>8);
  b[2]=(uint8_t)(v>>16); b[3]=(uint8_t)(v>>24); }
static inline void w64(uint8_t *b, uint64_t v)
{ w32(b,(uint32_t)v); w32(b+4,(uint32_t)(v>>32)); }
static inline uint32_t r32(const uint8_t *b)
{ return (uint32_t)b[0] | ((uint32_t)b[1]<<8)
       | ((uint32_t)b[2]<<16) | ((uint32_t)b[3]<<24); }

/* =========================================================================
 * Default output sink — blocking UART
 * ========================================================================= */
static void default_sink(const void *data, size_t len) {
    const uint8_t *p = (const uint8_t *)data;
    for (size_t i = 0; i < len; i++) {
        while (!serial_writable(&monitor_uart)) {}
        serial_putc(&monitor_uart, (int)p[i]);
    }
}
static volatile monitor_sink_cb_t s_sink = default_sink;
void monitor_set_output_sink(monitor_sink_cb_t cb) {
    s_sink = cb ? cb : default_sink;
}

/* =========================================================================
 * Lock-free packet pool — O(1) alloc/free via SPSC free-index ring.
 * ========================================================================= */
typedef struct { uint8_t buf[PACKET_BUFFER_SIZE]; } pool_slot_t;
static pool_slot_t s_pool[PACKET_POOL_SIZE];

static uint8_t           s_free_ring[PACKET_POOL_SIZE];
static volatile uint32_t s_free_rd  = 0u;
static volatile uint32_t s_free_wr  = 0u;

volatile uint32_t stats_captured       = 0u;
volatile uint32_t stats_dropped_ring   = 0u;
volatile uint32_t stats_dropped_pool   = 0u;
volatile uint32_t stats_dropped_block  = 0u;
volatile uint32_t stats_peak_pool_used = 0u;

static void pool_init(void) {
    for (uint32_t i = 0u; i < PACKET_POOL_SIZE; i++)
        s_free_ring[i] = (uint8_t)i;
    __atomic_store_n(&s_free_rd, 0u,               __ATOMIC_RELEASE);
    __atomic_store_n(&s_free_wr, PACKET_POOL_SIZE, __ATOMIC_RELEASE);
}

static uint8_t *pool_alloc(void)
{
    uint32_t rd = __atomic_load_n(&s_free_rd, __ATOMIC_RELAXED);
    uint32_t wr = __atomic_load_n(&s_free_wr, __ATOMIC_ACQUIRE);
    if (rd == wr) {
        __atomic_add_fetch(&stats_dropped_pool, 1u, __ATOMIC_RELAXED);
        return NULL;
    }

    uint8_t  idx    = s_free_ring[rd % PACKET_POOL_SIZE];
    uint32_t new_rd = rd + 1u;
    __atomic_store_n(&s_free_rd, new_rd, __ATOMIC_RELEASE);

    uint32_t used = PACKET_POOL_SIZE - (wr - new_rd);
    uint32_t old  = __atomic_load_n(&stats_peak_pool_used, __ATOMIC_RELAXED);
    while (used > old &&
           !__atomic_compare_exchange_n(&stats_peak_pool_used, &old, used,
                                        false, __ATOMIC_RELAXED,
                                        __ATOMIC_RELAXED)) {}
    return s_pool[idx].buf;
}

static void pool_free(uint8_t *buf) {
    if (!buf) return;
    uintptr_t idx = ((uintptr_t)buf - (uintptr_t)s_pool) / sizeof(pool_slot_t);
    if (idx >= (uintptr_t)PACKET_POOL_SIZE) return;

    uint32_t wr = __atomic_load_n(&s_free_wr, __ATOMIC_RELAXED);
    s_free_ring[wr % PACKET_POOL_SIZE] = (uint8_t)idx;
    __atomic_store_n(&s_free_wr, wr + 1u, __ATOMIC_RELEASE);
}

/* =========================================================================
 * Block pool — writer task; no ISR access
 * ========================================================================= */
typedef struct { uint8_t buf[BLOCK_MAX_SIZE]; uint32_t in_use; } block_slot_t;
static block_slot_t s_block_pool[BLOCK_POOL_SIZE];

static uint8_t *block_alloc(void) {
    for (int i = 0; i < (int)BLOCK_POOL_SIZE; i++) {
        uint32_t exp = 0u;
        if (__atomic_compare_exchange_n(&s_block_pool[i].in_use, &exp, 1u,
                                        false, __ATOMIC_ACQ_REL,
                                        __ATOMIC_ACQUIRE))
            return s_block_pool[i].buf;
    }
    return NULL;
}
static void block_free(uint8_t *buf) {
    if (!buf) return;
    uintptr_t idx = ((uintptr_t)buf - (uintptr_t)s_block_pool)
                    / sizeof(block_slot_t);
    if (idx < (uintptr_t)BLOCK_POOL_SIZE)
        __atomic_store_n(&s_block_pool[idx].in_use, 0u, __ATOMIC_RELEASE);
}

/* =========================================================================
 * Lock-free SPSC ring — promisc ISR (writer) → monitor_task (reader)
 * ========================================================================= */
typedef struct {
    uint8_t *data;
    uint32_t orig_len;
    uint32_t len;
    uint8_t  channel;
    int8_t   rssi;
    uint8_t  rate;
    uint64_t ts_ns;
} rb_entry_t;

static rb_entry_t        s_ring[RING_SIZE];
static volatile uint32_t s_write_idx = 0u;
static volatile uint32_t s_read_idx  = 0u;
static inline uint32_t ring_next(uint32_t i) { return (i+1u)&(RING_SIZE-1u); }

/* =========================================================================
 * Filters (applied in monitor_task, NOT in ISR)
 * ========================================================================= */
static volatile monitor_filter_t s_frame_filter = FILTER_ALL;
void monitor_set_filter(monitor_filter_t f) { s_frame_filter = f; }

static bool should_capture(const uint8_t *buf, uint32_t len) {
    if (s_frame_filter == FILTER_ALL) return true;
    if (len < 2u) return false;
    uint16_t fc  = (uint16_t)(buf[0]|((uint16_t)buf[1]<<8));
    uint8_t  ty  = (fc>>2)&0x3u, sub=(fc>>4)&0xFu;
    switch (s_frame_filter) {
        case FILTER_DATA:       return ty==2u;
        case FILTER_MANAGEMENT: return ty==0u;
        case FILTER_CONTROL:    return ty==1u;
        case FILTER_BEACON:     return ty==0u&&sub==8u;
        case FILTER_PROBE_REQ:  return ty==0u&&sub==4u;
        case FILTER_PROBE_RSP:  return ty==0u&&sub==5u;
        default:                return true;
    }
}

static volatile int8_t s_rssi_threshold = -128;
void monitor_set_rssi_threshold(int8_t min_rssi) { s_rssi_threshold = min_rssi; }

static volatile mac_filter_mode_t s_mac_mode = MAC_FILTER_NONE;
static uint8_t s_mac_list[MAX_MAC_ENTRIES][6];
static int     s_mac_count = 0;
void monitor_set_mac_filter_mode(mac_filter_mode_t m) { s_mac_mode = m; }
int monitor_add_mac_filter(const uint8_t *mac) {
    taskENTER_CRITICAL();
    if (s_mac_count>=(int)MAX_MAC_ENTRIES){taskEXIT_CRITICAL();return -1;}
    memcpy(s_mac_list[s_mac_count++], mac, 6u);
    taskEXIT_CRITICAL(); return 0;
}
int monitor_remove_mac_filter(const uint8_t *mac) {
    int ret=-1; taskENTER_CRITICAL();
    for(int i=0;i<s_mac_count;i++){
        if(memcmp(s_mac_list[i],mac,6u)==0){
            memmove(&s_mac_list[i],&s_mac_list[i+1],(size_t)(s_mac_count-i-1)*6u);
            s_mac_count--;ret=0;break;}}
    taskEXIT_CRITICAL();return ret;
}
void monitor_clear_mac_filter(void){taskENTER_CRITICAL();s_mac_count=0;taskEXIT_CRITICAL();}

static bool check_mac_filter(const uint8_t *a1, const uint8_t *a2,
                              const uint8_t *a3) {
    mac_filter_mode_t mode = s_mac_mode;
    if (mode==MAC_FILTER_NONE) return true;
    for(int i=0;i<s_mac_count;i++){
        const uint8_t *f=s_mac_list[i];
        if(memcmp(a1,f,6)==0||memcmp(a2,f,6)==0||memcmp(a3,f,6)==0)
            return (mode==MAC_FILTER_ALLOWLIST);}
    return (mode==MAC_FILTER_DENYLIST);
}

static monitor_bpf_rule_t s_bpf_rules[MAX_BPF_RULES];
static uint8_t s_bpf_count = 0u;
void monitor_set_bpf_rules(const monitor_bpf_rule_t *rules, uint8_t count) {
    uint8_t n=(count>(uint8_t)MAX_BPF_RULES)?(uint8_t)MAX_BPF_RULES:count;
    taskENTER_CRITICAL();
    s_bpf_count=n;
    if(n&&rules) memcpy(s_bpf_rules,rules,n*sizeof(*rules));
    taskEXIT_CRITICAL();
}
void monitor_clear_bpf_rules(void){taskENTER_CRITICAL();s_bpf_count=0;taskEXIT_CRITICAL();}
static bool check_bpf(const uint8_t *buf, uint32_t len) {
    for(uint8_t i=0;i<s_bpf_count;i++){
        if(s_bpf_rules[i].offset>=len) return false;
        if((buf[s_bpf_rules[i].offset]&s_bpf_rules[i].mask)!=s_bpf_rules[i].value)
            return false;}
    return true;
}

static volatile monitor_capture_cb_t s_capture_cb = NULL;
void monitor_set_capture_callback(monitor_capture_cb_t cb) { s_capture_cb = cb; }

/* =========================================================================
 * PROMISCUOUS CALLBACK — brutally minimal [ISR CONTRACT]
 * ========================================================================= */
static uint8_t promisc_callback(struct rtw_rx_pkt_info *pkt_info)
{
    if (!pkt_info || !pkt_info->buf || pkt_info->len == 0u) return 1u;

    uint64_t ts_ns = timer_get_time_ns_isr();
    uint32_t w    = __atomic_load_n(&s_write_idx, __ATOMIC_RELAXED);
    uint32_t r    = __atomic_load_n(&s_read_idx,  __ATOMIC_ACQUIRE);
    uint32_t next = ring_next(w);
    if (next == r) {
        __atomic_add_fetch(&stats_dropped_ring, 1u, __ATOMIC_RELAXED);
        return 1u;
    }

    uint8_t *buf = pool_alloc();
    if (!buf) return 1u;

    uint32_t copy_len = pkt_info->len;
    if (copy_len > PACKET_BUFFER_SIZE) copy_len = PACKET_BUFFER_SIZE;
    memcpy(buf, pkt_info->buf, copy_len);

    rb_entry_t *e = &s_ring[next];
    e->data     = buf;
    e->orig_len = pkt_info->len;
    e->len      = copy_len;
    e->channel  = pkt_info->channel;
    e->rssi     = (int8_t)pkt_info->recv_signal_power;
    e->rate     = (uint8_t)pkt_info->data_rate;
    e->ts_ns    = ts_ns;
    __atomic_store_n(&s_write_idx, next, __ATOMIC_RELEASE);
    __atomic_add_fetch(&stats_captured, 1u, __ATOMIC_RELAXED);
    return 1u;
}

/* =========================================================================
 * Radiotap builder (task context only)
 * ========================================================================= */
#if (MONITOR_FORMAT == MONITOR_FMT_PCAP) || (MONITOR_FORMAT == MONITOR_FMT_PCAPNG)
static uint16_t ch_to_freq(uint8_t ch) {
    if (ch>=1&&ch<=13) return (uint16_t)(2407u+5u*ch);
    if (ch==14) return 2484u;
    return (uint16_t)(5000u+5u*ch);
}
static size_t build_radiotap(uint8_t *out, size_t out_max,
                              uint8_t ch, int8_t rssi,
                              uint8_t rate_mgn, uint64_t tsf_us) {
    static const uint32_t PRESENT=(1u<<0)|(1u<<2)|(1u<<3)|(1u<<5);
    if (out_max < 28u) return 0u;

    memset(out, 0, out_max);

    out[0]=0; out[1]=0;
    w32(&out[4],PRESENT);
    size_t off=8u;
    w64(&out[off],tsf_us); off+=8u;
    out[off++]=rate_mgn;
    if(off&1u) out[off++]=0u;
    w16(&out[off],ch_to_freq(ch));
    w16(&out[off+2u],(ch<=14u)?0x0080u:0x0100u);
    off+=4u;
    out[off++]=(uint8_t)rssi;
    size_t padded=(off+3u)&~3u;
    while(off<padded) out[off++]=0u;
    w16(&out[2],(uint16_t)off);
    return off;
}
#endif

/* =========================================================================
 * Block queue helper
 * ========================================================================= */
static QueueHandle_t s_writer_queue = NULL;

static void queue_or_free(uint8_t *block) {
    if (s_writer_queue &&
        xQueueSend(s_writer_queue, &block, 0) == pdTRUE) return;
    block_free(block);
    __atomic_add_fetch(&stats_dropped_block, 1u, __ATOMIC_RELAXED);
}

/* =========================================================================
 * FORMAT: CAP / PCAP
 * ========================================================================= */
#if (MONITOR_FORMAT == MONITOR_FMT_CAP) || (MONITOR_FORMAT == MONITOR_FMT_PCAP)
#define PCAP_ENV_HDR 8u

static void write_pcap_global_header(void) {
    uint8_t h[24]={0};
    w32(&h[0],PCAP_MAGIC_US); w16(&h[4],2); w16(&h[6],4);
    w32(&h[8],0); w32(&h[12],0); w32(&h[16],PACKET_BUFFER_SIZE);
#if MONITOR_FORMAT == MONITOR_FMT_CAP
    w32(&h[20],LINKTYPE_IEEE802_11);
#else
    w32(&h[20],LINKTYPE_IEEE802_11_RADIOTAP);
#endif
    s_sink(h,sizeof(h));
}
static void pcap_queue_packet(const rb_entry_t *e, uint64_t tsf_us) {
#if MONITOR_FORMAT == MONITOR_FMT_PCAP
    uint8_t rt[32]; size_t rt_len=build_radiotap(rt,sizeof(rt),
        e->channel,e->rssi,e->rate,tsf_us);
    if (!rt_len) return;
#else
    (void)tsf_us; const uint8_t *rt=NULL; size_t rt_len=0u;
#endif
    uint32_t incl=(uint32_t)(rt_len+e->len);
    uint32_t orig=(uint32_t)(rt_len+e->orig_len);
    uint32_t total=(uint32_t)(PCAP_ENV_HDR+16u+incl);
    if (total>BLOCK_MAX_SIZE) return;
    uint8_t *block=block_alloc();
    if (!block){__atomic_add_fetch(&stats_dropped_block,1u,__ATOMIC_RELAXED);return;}
    w32(block+0,total); w32(block+4,total);
    uint8_t *rec=block+PCAP_ENV_HDR;
    w32(rec+0,(uint32_t)(e->ts_ns/1000000000ULL));
    w32(rec+4,(uint32_t)((e->ts_ns%1000000000ULL)/1000ULL));
    w32(rec+8,incl); w32(rec+12,orig);
    uint8_t *payload=rec+16u;
#if MONITOR_FORMAT == MONITOR_FMT_PCAP
    if(rt){memcpy(payload,rt,rt_len);payload+=rt_len;}
#endif
    memcpy(payload,e->data,e->len);
    queue_or_free(block);
}
static void writer_flush_block(uint8_t *block) {
    uint32_t total = r32(block);
    s_sink(block+PCAP_ENV_HDR, total-PCAP_ENV_HDR);
    block_free(block);
}
#define write_session_header()         write_pcap_global_header()
#define write_packet_record(e,tsf_us)  pcap_queue_packet(e,tsf_us)
#define writer_flush(b)                writer_flush_block(b)
#endif /* CAP || PCAP */

/* =========================================================================
 * FORMAT: PCAPNG
 * ========================================================================= */
#if MONITOR_FORMAT == MONITOR_FMT_PCAPNG
static void write_pcapng_shb(void) {
    uint8_t b[32]={0};
    w32(&b[0],0x0A0D0D0Au); w32(&b[4],sizeof(b)); w32(&b[8],0x1A2B3C4Du);
    w16(&b[12],1u); w16(&b[14],0u);
    w64(&b[16],0xFFFFFFFFFFFFFFFFULL);
    w32(&b[28],sizeof(b));
    s_sink(b,sizeof(b));
}
static void write_pcapng_idb(void) {
    uint8_t b[64]={0};
    size_t p=0;
    w32(&b[p],0x00000001u); p+=4;
    w32(&b[p],0);           p+=4;
    w16(&b[p],LINKTYPE_IEEE802_11_RADIOTAP); p+=2;
    w16(&b[p],0);           p+=2;
    w32(&b[p],PACKET_BUFFER_SIZE); p+=4;

    w16(&b[p],IDB_OPT_TSRESOL); p+=2;
    w16(&b[p],1u);          p+=2;
    b[p++]=IDB_TSRESOL_NS;
    b[p++]=0u; b[p++]=0u; b[p++]=0u;

    w16(&b[p],IDB_OPT_FCSLEN); p+=2;
    w16(&b[p],1u);          p+=2;
    b[p++]=IDB_FCS_LEN_BYTES;
    b[p++]=0u; b[p++]=0u; b[p++]=0u;

    w16(&b[p],0u); p+=2;
    w16(&b[p],0u); p+=2;

    configASSERT(p + 4u <= sizeof(b));
    uint32_t total=(uint32_t)(p+4u);
    w32(&b[4],total);
    w32(&b[p],total);
    p+=4;
    s_sink(b,p);
}
static void pcapng_queue_epb(const rb_entry_t *e, uint64_t tsf_us) {
    uint8_t rt[32];
    size_t rt_len=build_radiotap(rt,sizeof(rt),e->channel,e->rssi,
                                  e->rate,tsf_us);
    if (!rt_len) return;
    uint32_t caplen=(uint32_t)(rt_len+e->len);
    uint32_t padded=(caplen+3u)&~3u;
    uint32_t block_total=32u+padded;
    if (block_total>BLOCK_MAX_SIZE) return;
    uint8_t *block=block_alloc();
    if (!block){__atomic_add_fetch(&stats_dropped_block,1u,__ATOMIC_RELAXED);return;}
    w32(block+ 0,0x00000006u);
    w32(block+ 4,block_total);
    w32(block+ 8,0u);
    uint64_t ts=e->ts_ns;
    w32(block+12,(uint32_t)(ts>>32));
    w32(block+16,(uint32_t)ts);
    w32(block+20,caplen);
    w32(block+24,(uint32_t)(rt_len+e->orig_len));
    memcpy(block+28u,rt,rt_len);
    memcpy(block+28u+rt_len,e->data,e->len);
    if (padded>caplen) memset(block+28u+caplen,0u,padded-caplen);
    w32(block+28u+padded,block_total);
    queue_or_free(block);
}
static void writer_flush_block(uint8_t *block) {
    uint32_t total = r32(block + 4u);
    s_sink(block, total);
    block_free(block);
}
#define write_session_header()         do{write_pcapng_shb();write_pcapng_idb();}while(0)
#define write_packet_record(e,tsf_us)  pcapng_queue_epb(e,tsf_us)
#define writer_flush(b)                writer_flush_block(b)
#endif /* PCAPNG */

/* =========================================================================
 * Task state
 * ========================================================================= */
static volatile int  s_monitor_running     = 0;
static TaskHandle_t  s_monitor_handle      = NULL;
static TaskHandle_t  s_hopper_handle       = NULL;
static TaskHandle_t  s_writer_handle       = NULL;
static SemaphoreHandle_t s_monitor_done    = NULL;
static SemaphoreHandle_t s_hopper_done     = NULL;
static SemaphoreHandle_t s_writer_done     = NULL;
static SemaphoreHandle_t s_hopper_stop_sem = NULL;
static volatile uint8_t  s_fixed_ch        = 0u;
static volatile uint8_t  s_current_ch      = 0u;

/* =========================================================================
 * Writer task
 * ========================================================================= */
static void writer_task(void *param) {
    (void)param;
    uint8_t *block;
    while (s_monitor_running) {
        if (xQueueReceive(s_writer_queue, &block, pdMS_TO_TICKS(100u))==pdTRUE)
            writer_flush(block);
    }
    while (xQueueReceive(s_writer_queue, &block, 0)==pdTRUE)
        writer_flush(block);
    xSemaphoreGive(s_writer_done);
    vTaskDelete(NULL);
}

/* =========================================================================
 * Hopper task — adaptive dwell
 * ========================================================================= */
static void hopper_task(void *arg) {
    (void)arg;
    size_t idx = 0u;
    while (s_monitor_running) {
        uint8_t ch = (s_fixed_ch!=0u) ? s_fixed_ch : s_channel_list[idx];
        if (ch != s_current_ch) {
            wifi_set_channel(WLAN_IDX, ch);
            s_current_ch = ch;
        }
        uint32_t dwell_ms = HOP_DWELL_MIN_MS;
        int ci = channel_to_idx(ch);
        if (ci >= 0) {
            uint32_t ema = __atomic_load_n(&s_ch_stats[ci].traffic_ema, __ATOMIC_RELAXED);
            uint32_t thr = HOP_TRAFFIC_HIGH_THR * 256u;
            if (ema >= thr) dwell_ms = HOP_DWELL_MAX_MS;
            else dwell_ms = HOP_DWELL_MIN_MS +
                (uint32_t)(((uint64_t)(HOP_DWELL_MAX_MS - HOP_DWELL_MIN_MS)
                             * ema) / thr);
            uint32_t decayed = (ema * 7u) / 8u;
            __atomic_store_n(&s_ch_stats[ci].traffic_ema, decayed, __ATOMIC_RELAXED);
        }

        xSemaphoreTake(s_hopper_stop_sem, pdMS_TO_TICKS(dwell_ms));

        if (s_fixed_ch==0u) idx=(idx+1u)%CHANNEL_LIST_LEN;
    }
    xSemaphoreGive(s_hopper_done);
    vTaskDelete(NULL);
}

/* =========================================================================
 * Monitor task — drains ring, applies all filters, aggregates stats
 * ========================================================================= */
static void monitor_task(void *param) {
    (void)param;

    if (wifi_on(RTW_MODE_STA) != 0) {
        s_monitor_running = 0;
        xSemaphoreGive(s_hopper_done);
        xSemaphoreGive(s_monitor_done);
        vTaskDelete(NULL);
        return;
    }
    for (int w = 0; w < 5000; w += 200) {
        if (wifi_is_running(WLAN_IDX)) break;
        vTaskDelay(pdMS_TO_TICKS(200u));
    }

    struct rtw_promisc_para para = {0};
    para.filter_mode = 2u;
    para.callback    = promisc_callback;
    wifi_promisc_enable(1u, &para);

    xTaskCreate(hopper_task, "mon_hop",
                HOP_TASK_STACK / sizeof(StackType_t),
                NULL, HOP_TASK_PRIO, &s_hopper_handle);

    write_session_header();

    while (s_monitor_running) {
        uint32_t r = __atomic_load_n(&s_read_idx,  __ATOMIC_RELAXED);
        uint32_t w = __atomic_load_n(&s_write_idx, __ATOMIC_ACQUIRE);

        uint64_t tsf_cached = 0u;
        if (r != w) {
            wifi_get_tsf(WLAN_IDX, &tsf_cached);
        }

        while (r != w) {
            uint32_t   next = ring_next(r);
            rb_entry_t entry;
            memcpy(&entry, &s_ring[next], sizeof(entry));
            __atomic_store_n(&s_read_idx, next, __ATOMIC_RELEASE);

            if (!entry.data || entry.len == 0u) { r=next; continue; }

            bool pass = (entry.rssi >= s_rssi_threshold)
                     && should_capture(entry.data, entry.len);
            if (pass && s_mac_mode!=MAC_FILTER_NONE && entry.len>=24u)
                pass = check_mac_filter(entry.data+4u,
                                        entry.data+10u,
                                        entry.data+16u);
            if (pass && s_bpf_count > 0u)
                pass = check_bpf(entry.data, entry.len);

            if (pass) {
                uint8_t fc_type = (entry.len>=2u)
                                  ? ((entry.data[0]>>2)&0x3u) : 3u;
                ch_stat_update(entry.channel, entry.rssi, fc_type);

                monitor_capture_cb_t cb = s_capture_cb;
                if (cb) cb(entry.data, entry.len, entry.channel,
                           entry.rssi, entry.ts_ns);

                write_packet_record(&entry, (uint64_t)tsf_cached);
            }
            pool_free(entry.data);
            r = next;
            w = __atomic_load_n(&s_write_idx, __ATOMIC_ACQUIRE);
        }
        vTaskDelay(pdMS_TO_TICKS(1u));
    }

    {
        uint32_t r2=__atomic_load_n(&s_read_idx,__ATOMIC_RELAXED);
        uint32_t w2=__atomic_load_n(&s_write_idx,__ATOMIC_ACQUIRE);
        while(r2!=w2){
            uint32_t next=ring_next(r2);
            if(s_ring[next].data) pool_free(s_ring[next].data);
            __atomic_store_n(&s_read_idx,next,__ATOMIC_RELEASE);
            r2=next;}
    }

    xSemaphoreGive(s_monitor_done);
    vTaskDelete(NULL);
}

/* =========================================================================
 * Global statistics
 * ========================================================================= */
void monitor_get_stats(uint32_t *cap, uint32_t *dr, uint32_t *dp,
                       uint32_t *pk, uint32_t *db) {
    *cap = __atomic_load_n(&stats_captured,       __ATOMIC_RELAXED);
    *dr  = __atomic_load_n(&stats_dropped_ring,   __ATOMIC_RELAXED);
    *dp  = __atomic_load_n(&stats_dropped_pool,   __ATOMIC_RELAXED);
    *pk  = __atomic_load_n(&stats_peak_pool_used, __ATOMIC_RELAXED);
    *db  = __atomic_load_n(&stats_dropped_block,  __ATOMIC_RELAXED);
}
void monitor_reset_stats(void) {
    __atomic_store_n(&stats_captured,       0u,__ATOMIC_RELAXED);
    __atomic_store_n(&stats_dropped_ring,   0u,__ATOMIC_RELAXED);
    __atomic_store_n(&stats_dropped_pool,   0u,__ATOMIC_RELAXED);
    __atomic_store_n(&stats_dropped_block,  0u,__ATOMIC_RELAXED);
    __atomic_store_n(&stats_peak_pool_used, 0u,__ATOMIC_RELAXED);
}

/* =========================================================================
 * Public control
 * ========================================================================= */
void monitor_start(void) {
    if (s_monitor_handle) return;

    pool_init();

    s_monitor_done    = xSemaphoreCreateBinary();
    s_hopper_done     = xSemaphoreCreateBinary();
    s_writer_done     = xSemaphoreCreateBinary();
    s_hopper_stop_sem = xSemaphoreCreateBinary();
    s_writer_queue    = xQueueCreate(WRITER_QUEUE_LEN, sizeof(uint8_t *));
    configASSERT(s_monitor_done && s_hopper_done && s_writer_done
                 && s_hopper_stop_sem && s_writer_queue);

    s_monitor_running = 1;

    xTaskCreate(writer_task,  "mon_wr",
                WRITER_TASK_STACK/sizeof(StackType_t),
                NULL, WRITER_TASK_PRIO, &s_writer_handle);
    xTaskCreate(monitor_task, "mon_rx",
                MONITOR_TASK_STACK/sizeof(StackType_t),
                NULL, MONITOR_TASK_PRIO, &s_monitor_handle);
}

void monitor_stop(void) {
    if (!s_monitor_handle) return;
    s_monitor_running = 0;
    wifi_promisc_enable(0u, NULL);

    if (s_hopper_stop_sem) xSemaphoreGive(s_hopper_stop_sem);

    if (s_monitor_done) {
        xSemaphoreTake(s_monitor_done, pdMS_TO_TICKS(MONITOR_JOIN_TIMEOUT_MS));
        vSemaphoreDelete(s_monitor_done); s_monitor_done=NULL;
    }
    if (s_hopper_done) {
        xSemaphoreTake(s_hopper_done, pdMS_TO_TICKS(HOP_JOIN_TIMEOUT_MS));
        vSemaphoreDelete(s_hopper_done); s_hopper_done=NULL;
    }
    if (s_hopper_stop_sem) {
        vSemaphoreDelete(s_hopper_stop_sem); s_hopper_stop_sem=NULL;
    }
    if (s_writer_done) {
        xSemaphoreTake(s_writer_done, pdMS_TO_TICKS(WRITER_JOIN_TIMEOUT_MS));
        vSemaphoreDelete(s_writer_done); s_writer_done=NULL;
    }
    s_monitor_handle=s_hopper_handle=s_writer_handle=NULL;

    if (s_writer_queue) {
        uint8_t *stale;
        while(xQueueReceive(s_writer_queue,&stale,0)==pdTRUE) block_free(stale);
        vQueueDelete(s_writer_queue); s_writer_queue=NULL;
    }

    pool_init();
    for(int i=0;i<(int)BLOCK_POOL_SIZE;i++)
        __atomic_store_n(&s_block_pool[i].in_use,0u,__ATOMIC_RELEASE);
}

void    monitor_set_fixed_channel(uint8_t ch) { s_fixed_ch=ch; }
void    monitor_set_hopping(void)             { s_fixed_ch=0u; }
uint8_t monitor_get_channel(void)             { return s_current_ch; }
