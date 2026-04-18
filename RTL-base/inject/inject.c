/**
 * @file  inject.c
 * @brief Wi-Fi Packet Injector — RTL8721Dx / KM4.
 *        Scheduler: Channel-Aware EDF (CA-EDF) with EMA-derived slack batching.
 */

#include "inject.h"
#include "timer.h"
#include <string.h>
#include <stdio.h>
#include <limits.h>
#include "FreeRTOS.h"
#include "task.h"
#include "wifi_api.h"
#include "wifi_api_ext.h"
#include "wifi_api_types.h"

/* =========================================================================
 * Compile-time assertions
 * ========================================================================= */
_Static_assert(INJECTOR_MAX > 0u && INJECTOR_MAX <= 32u,
               "INJECTOR_MAX must be in [1, 32]");
_Static_assert(INJECTOR_MAX_PACKET_SIZE >= 64u,
               "INJECTOR_MAX_PACKET_SIZE too small");
_Static_assert(INJECTOR_HIST_BINS == 8u,
               "Histogram code assumes exactly 8 bins");
_Static_assert(INJECTOR_BACKOFF_MAX_STEPS <= 31u,
               "Backoff shift would overflow uint64_t");

/* =========================================================================
 * Logging
 * ========================================================================= */
static inj_log_cb_t s_log_cb = NULL;
void injectorManager_setLogCallback(inj_log_cb_t cb) { s_log_cb = cb; }

#define INJ_LOG(fmt, ...) \
    do { if (s_log_cb) { char _b[256]; \
         snprintf(_b, sizeof(_b), fmt, ##__VA_ARGS__); s_log_cb(_b); } \
         else { printf(fmt, ##__VA_ARGS__); } } while (0)

/* =========================================================================
 * AC → hardware queue  (AN1003: ac_queue field of rtw_raw_frame_desc)
 * ========================================================================= */
static const uint8_t s_ac_hw[4] = { 0u, 1u, 4u, 6u };

/* =========================================================================
 * Per-band switch overhead EMA
 * ========================================================================= */
#define SW_SAME  0
#define SW_CROSS 1

static uint64_t s_switch_ema_ns[2] = { 10000000ULL, 20000000ULL };

static int switch_band_idx(uint8_t from, uint8_t to) {
    return ((from >= 36u) == (to >= 36u)) ? SW_SAME : SW_CROSS;
}

static void update_switch_ema(uint8_t from, uint8_t to, uint64_t ns) {
    if (from == 0u || to == 0u) return;
    int idx = switch_band_idx(from, to);
    s_switch_ema_ns[idx] = (s_switch_ema_ns[idx] * 7u + ns) / 8u;
}

static uint64_t predict_switch_cost_ns(uint8_t from, uint8_t to) {
    return (from == to) ? 0ULL :
           s_switch_ema_ns[switch_band_idx(from, to)];
}

static uint64_t switch_slack_window_ns(uint8_t from, uint8_t to) {
    if (from == to) return 0ULL;
    uint64_t cost = s_switch_ema_ns[switch_band_idx(from, to)];
    return cost + (cost >> 1u);
}

/* =========================================================================
 * Integer sqrt (Newton) for stddev approximation
 * ========================================================================= */
static uint64_t isqrt64(uint64_t n) {
    if (!n) return 0ULL;
    uint64_t x = n, y = (n + 1u) / 2u;
    while (y < x) { x = y; y = (x + n / x) / 2u; }
    return x;
}

/* =========================================================================
 * Air-time estimator — simplified CCK / OFDM / HT PHY model
 * ========================================================================= */
uint32_t injectorManager_estimateAirTimeUs(inject_rate_t rate, uint32_t frame_bytes)
{
    uint32_t preamble_us, kbps;
    switch (rate) {
        case INJ_RATE_1M:   preamble_us=144; kbps=  1000; break;
        case INJ_RATE_2M:   preamble_us=144; kbps=  2000; break;
        case INJ_RATE_5_5M: preamble_us= 72; kbps=  5500; break;
        case INJ_RATE_11M:  preamble_us= 72; kbps= 11000; break;
        case INJ_RATE_6M:   preamble_us= 36; kbps=  6000; break;
        case INJ_RATE_9M:   preamble_us= 36; kbps=  9000; break;
        case INJ_RATE_12M:  preamble_us= 36; kbps= 12000; break;
        case INJ_RATE_18M:  preamble_us= 36; kbps= 18000; break;
        case INJ_RATE_24M:  preamble_us= 36; kbps= 24000; break;
        case INJ_RATE_36M:  preamble_us= 36; kbps= 36000; break;
        case INJ_RATE_48M:  preamble_us= 36; kbps= 48000; break;
        case INJ_RATE_54M:  preamble_us= 36; kbps= 54000; break;
        case INJ_RATE_MCS0: preamble_us= 40; kbps=  6500; break;
        case INJ_RATE_MCS1: preamble_us= 40; kbps= 13000; break;
        case INJ_RATE_MCS2: preamble_us= 40; kbps= 19500; break;
        case INJ_RATE_MCS3: preamble_us= 40; kbps= 26000; break;
        case INJ_RATE_MCS4: preamble_us= 40; kbps= 39000; break;
        case INJ_RATE_MCS5: preamble_us= 40; kbps= 52000; break;
        case INJ_RATE_MCS6: preamble_us= 40; kbps= 58500; break;
        case INJ_RATE_MCS7: preamble_us= 40; kbps= 65000; break;
        default:            preamble_us= 36; kbps=  6000; break;
    }
    uint32_t payload_us = (uint32_t)(
        ((uint64_t)frame_bytes * 8u * 1000u + kbps - 1u) / kbps);
    return preamble_us + payload_us;
}

/* =========================================================================
 * Internal injector state
 * ========================================================================= */
typedef enum {
    INJ_STATE_IDLE=0, INJ_STATE_READY, INJ_STATE_SENDING, INJ_STATE_DONE
} injector_state_t;

typedef struct {
    char             name[INJECTOR_NAME_MAX];
    bool             in_use;
    bool             active;
    uint8_t          channel;
    uint8_t          hwRetries;
    uint8_t          swRetries;
    uint8_t          swRetries_attempted;
    uint8_t          backoff_step;
    uint8_t          priority;
    int8_t           tx_power_dbm;
    inject_rate_t    tx_rate;
    inject_flags_t   flags;
    inject_ac_t      ac_queue;
    uint32_t         burst_count;
    uint64_t         interval_ns;
    uint64_t         start_ns;
    uint64_t         burst_idx;
    uint64_t         next_desired_ns;
    uint32_t         maxPackets;
    uint32_t         packets_sent;
    uint32_t         seq_num;
    uint32_t         tx_errors;
    uint32_t         swRetries_total;
    uint64_t         total_air_time_us;
    uint8_t         *packetData;
    uint32_t         packetLen;
    injector_state_t state;
    uint32_t         generation;

    inj_mutate_cb_t  mutate_cb;
    void            *mutate_ctx;

    bool     stat_init;
    int64_t  stat_mean_ns;
    uint64_t stat_var_ns2;
    uint64_t stat_stddev_ns;
    int64_t  stat_min_ns;
    int64_t  stat_max_ns;
    uint64_t stat_switch_mean_ns;
    uint32_t stat_misses;
    uint32_t stat_skips;
    uint32_t stat_bursts_ok;
    uint32_t stat_histogram[INJECTOR_HIST_BINS];
} PacketInjector;

/* =========================================================================
 * Manager structure
 * ========================================================================= */
struct injectorManager {
    PacketInjector    injectors[INJECTOR_MAX];
    uint8_t           sched_snap_buf[INJECTOR_MAX_PACKET_SIZE];

    uint8_t           injectorCount;
    uint64_t          totalPacketsAllTime;
    uint32_t          totalErrorsAllTime;
    int8_t            current_tx_power;
    uint8_t           current_channel;
    uint8_t           wlan_idx;

    SemaphoreHandle_t lock;
    SemaphoreHandle_t wake_sem;
    SemaphoreHandle_t stop_done_sem;
    TaskHandle_t      scheduler_task;

    volatile inj_sched_state_t sched_state;

    uint8_t (*power_map_cb)(int8_t dbm);
};

/* =========================================================================
 * Forward declarations
 * ========================================================================= */
static void schedulerTask(void *param);
static int  stop_scheduler_internal(injectorManager *mgr);

/* =========================================================================
 * Validation helpers
 * ========================================================================= */
static bool is_valid_channel(uint8_t ch) {
    return ch == 0u || (ch >= 1u && ch <= 13u) || (ch >= 36u && ch <= 165u);
}

static bool is_valid_rate(inject_rate_t r) {
    switch (r) {
        case INJ_RATE_1M: case INJ_RATE_2M:   case INJ_RATE_5_5M:
        case INJ_RATE_11M:case INJ_RATE_6M:   case INJ_RATE_9M:
        case INJ_RATE_12M:case INJ_RATE_18M:  case INJ_RATE_24M:
        case INJ_RATE_36M:case INJ_RATE_48M:  case INJ_RATE_54M:
        case INJ_RATE_MCS0:case INJ_RATE_MCS1:case INJ_RATE_MCS2:
        case INJ_RATE_MCS3:case INJ_RATE_MCS4:case INJ_RATE_MCS5:
        case INJ_RATE_MCS6:case INJ_RATE_MCS7: return true;
        default: return false;
    }
}

/* =========================================================================
 * TX power
 * ========================================================================= */
static uint8_t default_dbm_to_pct(int8_t dbm) {
    if (dbm <=  0) return 13u; if (dbm <=  5) return 25u;
    if (dbm <= 10) return 50u; if (dbm <= 15) return 75u;
    return 100u;
}

void injectorManager_setPowerMappingCallback(injectorManager *mgr,
                                              uint8_t (*cb)(int8_t dbm)) {
    if (!mgr) return;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    mgr->power_map_cb = cb;
    xSemaphoreGive(mgr->lock);
}

__attribute__((weak)) int wifi_set_tx_power_percentage(uint8_t i, uint8_t p)
{ (void)i; (void)p; return 0; }

static int apply_tx_power(uint8_t wlan_idx, int8_t dbm, uint8_t (*map)(int8_t)) {
    if (dbm < 0) return 0;
    return wifi_set_tx_power_percentage(wlan_idx,
                                         map ? map(dbm) : default_dbm_to_pct(dbm));
}

/* =========================================================================
 * Internal helpers
 * ========================================================================= */
static PacketInjector *find_by_name(injectorManager *mgr, const char *name) {
    for (int i = 0; i < (int)INJECTOR_MAX; i++) {
        PacketInjector *p = &mgr->injectors[i];
        if (p->in_use && strcmp(p->name, name) == 0) return p;
    }
    return NULL;
}

static PacketInjector *find_free_slot(injectorManager *mgr) {
    for (int i = 0; i < (int)INJECTOR_MAX; i++)
        if (!mgr->injectors[i].in_use) return &mgr->injectors[i];
    return NULL;
}

static void delete_injector(injectorManager *mgr, PacketInjector *inj) {
    if (!inj->in_use) return;
    inj->active = false; inj->state = INJ_STATE_IDLE;
    if (inj->packetData) { vPortFree(inj->packetData); inj->packetData = NULL; }
    inj->generation++; inj->in_use = false; inj->name[0] = '\0';
    mgr->injectorCount--;
}

/* =========================================================================
 * 802.11 Sequence Control patch (bytes 22-23, fragment nibble preserved)
 * ========================================================================= */
static void patch_seq_ctrl(uint8_t *frame, uint32_t len, uint32_t seq) {
    if (len < 24u) return;
    uint8_t  frag = frame[22] & 0x0Fu;
    uint16_t sc   = (uint16_t)(((seq & 0xFFFu) << 4) | frag);
    frame[22] = (uint8_t)(sc & 0xFFu);
    frame[23] = (uint8_t)(sc >> 8);
}

/* =========================================================================
 * Packet transmission
 * ========================================================================= */
static int sendPacket(const uint8_t  *buf,
                       uint32_t        len,
                       uint8_t         wlan_idx,
                       inject_rate_t   rate,
                       uint8_t         hwRetries,
                       inject_ac_t     ac,
                       inject_flags_t  flags)
{
    configASSERT(buf != NULL);
    configASSERT(len > 0u && len <= INJECTOR_MAX_PACKET_SIZE);

    struct rtw_raw_frame_desc desc = {0};
    desc.wlan_idx    = wlan_idx;
    desc.device_id   = 0u;
    desc.buf         = (u8 *)buf;
    desc.buf_len     = (u16)len;
    desc.tx_rate     = (u8)rate;
    desc.retry_limit = hwRetries;
    desc.ac_queue    = s_ac_hw[ac & 3u];
    desc.sgi         = (flags & INJ_FLAG_USE_SHORT_GI) ? 1u : 0u;
    desc.agg_en      = (flags & INJ_FLAG_AGGREGATE)    ? 1u : 0u;
    return (int)wifi_send_raw_frame(&desc);
}

/* =========================================================================
 * EWMA online statistics + 8-bin jitter histogram
 * ========================================================================= */
static void stat_update(PacketInjector *inj, int64_t error_ns, uint64_t sw_ns)
{
    if (!inj->stat_init) {
        inj->stat_mean_ns        = error_ns;
        inj->stat_var_ns2        = 0ULL;
        inj->stat_stddev_ns      = 0ULL;
        inj->stat_min_ns         = error_ns;
        inj->stat_max_ns         = error_ns;
        inj->stat_switch_mean_ns = sw_ns;
        inj->stat_init           = true;
    } else {
        int64_t old_mean = inj->stat_mean_ns;
        inj->stat_mean_ns = (old_mean * 7 + error_ns) / 8;

        int64_t delta1 = error_ns - old_mean;
        int64_t delta2 = error_ns - inj->stat_mean_ns;
        uint64_t abs1  = (uint64_t)(delta1 < 0 ? -delta1 : delta1);
        uint64_t abs2  = (uint64_t)(delta2 < 0 ? -delta2 : delta2);
        if (abs1 > 4294967295ULL) abs1 = 4294967295ULL;
        if (abs2 > 4294967295ULL) abs2 = 4294967295ULL;
        uint64_t cross = abs1 * abs2;

        inj->stat_var_ns2   = (inj->stat_var_ns2 * 7u + cross) / 8u;
        inj->stat_stddev_ns = isqrt64(inj->stat_var_ns2);
        if (error_ns < inj->stat_min_ns) inj->stat_min_ns = error_ns;
        if (error_ns > inj->stat_max_ns) inj->stat_max_ns = error_ns;
        inj->stat_switch_mean_ns = (inj->stat_switch_mean_ns * 7u + sw_ns) / 8u;
    }

    int64_t offset = error_ns + (int64_t)((INJECTOR_HIST_BINS / 2u)
                                           * INJECTOR_HIST_BIN_WIDTH_NS);
    int bin = (offset < 0) ? 0
            : (int)(offset / INJECTOR_HIST_BIN_WIDTH_NS);
    if (bin >= (int)INJECTOR_HIST_BINS) bin = (int)INJECTOR_HIST_BINS - 1;
    inj->stat_histogram[bin]++;
}

/* =========================================================================
 * 99th-percentile adaptive spin guard: mean + 3σ, clamped
 * ========================================================================= */
static uint64_t adaptive_guard(const PacketInjector *inj) {
    if (!inj->stat_init) return INJECTOR_SPIN_GUARD_MIN_NS;
    int64_t  mean = inj->stat_mean_ns < 0 ? 0 : inj->stat_mean_ns;
    uint64_t g    = (uint64_t)mean + 3u * inj->stat_stddev_ns;
    if (g < INJECTOR_SPIN_GUARD_MIN_NS) g = INJECTOR_SPIN_GUARD_MIN_NS;
    if (g > INJECTOR_SPIN_GUARD_MAX_NS) g = INJECTOR_SPIN_GUARD_MAX_NS;
    return g;
}

/* =========================================================================
 * EDF min-heap
 * ========================================================================= */
typedef struct {
    PacketInjector *inj;
    uint64_t        deadline_ns;
    uint64_t        sort_key;
} SchedEntry;

static void hs(SchedEntry *a, SchedEntry *b) { SchedEntry t=*a;*a=*b;*b=t; }
static void sift_up(SchedEntry *h, int i) {
    while (i>0){int p=(i-1)/2;if(h[p].sort_key<=h[i].sort_key)break;
        hs(&h[p],&h[i]);i=p;} }
static void sift_dn(SchedEntry *h, int n, int i) {
    for(;;){int l=2*i+1,r=2*i+2,m=i;
        if(l<n&&h[l].sort_key<h[m].sort_key)m=l;
        if(r<n&&h[r].sort_key<h[m].sort_key)m=r;
        if(m==i)break;hs(&h[m],&h[i]);i=m;} }
static void heap_push(SchedEntry *h, int *n, SchedEntry e) {
    if(*n>=(int)INJECTOR_MAX)return; h[(*n)++]=e; sift_up(h,*n-1); }
static __attribute__((unused))
SchedEntry heap_pop(SchedEntry *h, int *n) {
    SchedEntry t=h[0]; if(--(*n)>0){h[0]=h[*n];sift_dn(h,*n,0);} return t; }
static uint64_t edf_sort_key(const PacketInjector *inj, uint8_t cur_ch) {
    uint64_t key = inj->next_desired_ns;
    if (!(inj->flags & INJ_FLAG_FIXED_CHANNEL) &&
         inj->channel != 0u && inj->channel != cur_ch)
        key += predict_switch_cost_ns(cur_ch, inj->channel);
    if (inj->priority > 0u) {
        uint64_t base  = (inj->interval_ns > 0u)
                       ? inj->interval_ns
                       : INJECTOR_DEADLINE_WINDOW_NS;
        uint64_t bonus = ((uint64_t)inj->priority * base) / 256u;
        key = (bonus < key) ? key - bonus : 0ULL;
    }
    return key;
}

/* =========================================================================
 * SCHEDULER TASK
 * ========================================================================= */
static void schedulerTask(void *param)
{
    injectorManager *mgr = (injectorManager *)param;
    SchedEntry heap[INJECTOR_MAX];
    int        heap_n = 0;

    for (;;) {
        if (mgr->sched_state != INJ_SCHED_RUNNING) break;

        uint64_t now = timer_get_time_ns_fine();

        xSemaphoreTake(mgr->lock, portMAX_DELAY);
        heap_n = 0;
        uint8_t cur_ch = mgr->current_channel;
        for (int i = 0; i < (int)INJECTOR_MAX; i++) {
            PacketInjector *inj = &mgr->injectors[i];
            if (!inj->in_use || !inj->active || inj->state != INJ_STATE_READY)
                continue;
            configASSERT(inj->packetData != NULL);
            SchedEntry e = { inj, inj->next_desired_ns,
                             edf_sort_key(inj, cur_ch) };
            heap_push(heap, &heap_n, e);
        }
        xSemaphoreGive(mgr->lock);

        if (heap_n == 0) {
            xSemaphoreTake(mgr->wake_sem, portMAX_DELAY);
            continue;
        }

        uint64_t earliest = heap[0].deadline_ns;
        xSemaphoreTake(mgr->lock, portMAX_DELAY);
        uint64_t guard = adaptive_guard(heap[0].inj);
        xSemaphoreGive(mgr->lock);

        if (earliest > now + guard) {
            uint64_t  sleep_ns = earliest - now - guard;
            TickType_t ticks   = pdMS_TO_TICKS((sleep_ns + 999999ULL) / 1000000ULL);
            if (ticks > 0) xSemaphoreTake(mgr->wake_sem, ticks);
            else taskYIELD();
            continue;
        }

        uint8_t         ca_edf_ch     = heap[0].inj->channel;
        uint64_t        ca_slack      = switch_slack_window_ns(cur_ch, ca_edf_ch);
        uint64_t        ca_l1_thresh  = ca_slack + (ca_slack >> 1u);
        uint64_t        ca_now        = timer_get_time_ns_fine();
        PacketInjector *ca_winner     = heap[0].inj;

        if (earliest > ca_now + ca_l1_thresh) {
            uint64_t slack_lim = ca_now + ca_slack;
            uint64_t best_dl   = UINT64_MAX;

            xSemaphoreTake(mgr->lock, portMAX_DELAY);
            for (int i = 0; i < (int)INJECTOR_MAX; i++) {
                PacketInjector *p = &mgr->injectors[i];
                if (!p->in_use || !p->active || p->state != INJ_STATE_READY)
                    continue;
                if (p->channel != cur_ch && p->channel != 0u)
                    continue;
                if (p->next_desired_ns <= slack_lim &&
                    p->next_desired_ns <  best_dl) {
                    best_dl   = p->next_desired_ns;
                    ca_winner = p;
                }
            }
            xSemaphoreGive(mgr->lock);
            timer_spin_until_ns(
                (ca_winner != heap[0].inj) ? best_dl : earliest);
        } else {
            timer_spin_until_ns(earliest);
        }

        xSemaphoreTake(mgr->lock, portMAX_DELAY);
        PacketInjector *inj = ca_winner;

        if (mgr->sched_state != INJ_SCHED_RUNNING) {
            xSemaphoreGive(mgr->lock); break;
        }

        if (!inj->in_use || !inj->active || inj->state != INJ_STATE_READY) {
            xSemaphoreGive(mgr->lock); continue;
        }
        configASSERT(inj->packetData != NULL);
        configASSERT(inj->packetLen > 0u && inj->packetLen <= INJECTOR_MAX_PACKET_SIZE);

        uint64_t now2        = timer_get_time_ns_fine();
        uint64_t interval_ns = inj->interval_ns;

        if ((inj->flags & INJ_FLAG_SKIP_IF_LATE) && interval_ns > 0u &&
            now2 > inj->next_desired_ns + interval_ns / 2u) {
            inj->stat_skips++;
            inj->swRetries_attempted = 0u;
            inj->backoff_step        = 0u;
            uint64_t elapsed         = now2 - inj->start_ns;
            uint64_t new_idx         = elapsed / interval_ns + 1u;
            inj->burst_idx           = new_idx;
            inj->next_desired_ns     = inj->start_ns + new_idx * interval_ns;
            inj->state = INJ_STATE_READY;
            xSemaphoreGive(mgr->lock); continue;
        }

        uint64_t       deadline_snap  = inj->next_desired_ns;
        uint64_t       burst_idx_snap = inj->burst_idx;
        uint64_t       start_snap     = inj->start_ns;
        uint32_t       burst_n        = (inj->burst_count < 1u) ? 1u : inj->burst_count;
        uint32_t       maxPkts        = inj->maxPackets;
        uint32_t       already_sent   = inj->packets_sent;
        uint32_t       gen_snap       = inj->generation;
        uint32_t       seq_snap       = inj->seq_num;
        uint8_t        target_ch      = inj->channel;
        int8_t         target_pw      = inj->tx_power_dbm;
        uint32_t       pkt_len        = inj->packetLen;
        inject_rate_t  snap_rate      = inj->tx_rate;
        uint8_t        snap_hw        = inj->hwRetries;
        inject_ac_t    snap_ac        = inj->ac_queue;
        inject_flags_t snap_flags     = inj->flags;
        inj_mutate_cb_t mutate_cb     = inj->mutate_cb;
        void           *mutate_ctx    = inj->mutate_ctx;
        uint8_t        wlan_idx       = mgr->wlan_idx;
        uint8_t        mgr_cur_ch     = mgr->current_channel;
        int8_t         mgr_cur_pw     = mgr->current_tx_power;

        memcpy(mgr->sched_snap_buf, inj->packetData, pkt_len);
        inj->state = INJ_STATE_SENDING;
        xSemaphoreGive(mgr->lock);

        uint64_t t_sw  = timer_get_time_ns_fine();
        uint8_t  sw_to = mgr_cur_ch;
        if (!(snap_flags & INJ_FLAG_FIXED_CHANNEL) &&
             target_ch != 0u && target_ch != mgr_cur_ch) {
            uint64_t sw_window    = switch_slack_window_ns(mgr_cur_ch, target_ch);
            uint64_t switch_at_ns = (deadline_snap > sw_window)
                                  ? deadline_snap - sw_window : 0ULL;
            uint64_t sw_now       = timer_get_time_ns_fine();
            if (switch_at_ns > sw_now) {
                uint64_t  sleep_ns = switch_at_ns - sw_now;
                TickType_t ticks   = pdMS_TO_TICKS((sleep_ns + 999999ULL) / 1000000ULL);
                if (ticks > 0) vTaskDelay(ticks);
                timer_spin_until_ns(switch_at_ns);
            }

            t_sw = timer_get_time_ns_fine();
            if (wifi_set_channel(wlan_idx, target_ch) == 0) {
                sw_to = target_ch;
                xSemaphoreTake(mgr->lock, portMAX_DELAY);
                mgr->current_channel = target_ch;
                xSemaphoreGive(mgr->lock);
            }
        }
        uint64_t sw_ns = timer_get_time_ns_fine() - t_sw;
        if (sw_to != mgr_cur_ch) update_switch_ema(mgr_cur_ch, sw_to, sw_ns);

        if (target_pw >= 0 && target_pw != mgr_cur_pw) {
            if (apply_tx_power(wlan_idx, target_pw, mgr->power_map_cb) == 0) {
                xSemaphoreTake(mgr->lock, portMAX_DELAY);
                mgr->current_tx_power = target_pw;
                xSemaphoreGive(mgr->lock);
            }
        }

        uint32_t burst_ok  = 0u;
        bool     had_error = false;

        for (uint32_t b = 0u; b < burst_n; b++) {
            if (mgr->sched_state != INJ_SCHED_RUNNING) {
                had_error = true; break;
            }
            if (maxPkts > 0u && (already_sent + burst_ok) >= maxPkts) break;

            if (b > 0u && interval_ns > 0u)
                timer_spin_until_ns(start_snap + (burst_idx_snap + b) * interval_ns);

            uint8_t *frame = mgr->sched_snap_buf;
            if (snap_flags & INJ_FLAG_AUTO_SEQ)
                patch_seq_ctrl(frame, pkt_len, (seq_snap + burst_ok) & 0xFFFu);
            if (mutate_cb)
                mutate_cb(frame, pkt_len, seq_snap + burst_ok, mutate_ctx);

            int r = sendPacket(frame, pkt_len, wlan_idx,
                               snap_rate, snap_hw, snap_ac, snap_flags);
            if (r == 0) { burst_ok++; }
            else {
                INJ_LOG("inject: [%s] wifi_send_raw_frame error %d "
                        "(burst %u/%u, attempt %u)\n",
                        inj->name, r,
                        (unsigned int)(b + 1u),
                        (unsigned int)burst_n,
                        (unsigned int)(inj->swRetries_attempted + 1u));
                had_error = true;
                break;
            }
        }

        xSemaphoreTake(mgr->lock, portMAX_DELAY);

        if (!inj->in_use || inj->generation != gen_snap) {
            if (inj->in_use && inj->state == INJ_STATE_SENDING)
                inj->state = INJ_STATE_READY;
            xSemaphoreGive(mgr->lock); continue;
        }

        int64_t sched_error = (int64_t)timer_get_time_ns_fine()
                            - (int64_t)deadline_snap;
        stat_update(inj, sched_error, sw_ns);
        if (sched_error > 0) inj->stat_misses++;

        inj->packets_sent        += burst_ok;
        inj->seq_num              = (inj->seq_num + burst_ok) & 0xFFFu;
        mgr->totalPacketsAllTime += burst_ok;
        inj->total_air_time_us   +=
            (uint64_t)injectorManager_estimateAirTimeUs(snap_rate, pkt_len)
            * burst_ok;

        if (had_error) {
            inj->swRetries_attempted++;
            inj->swRetries_total++;
            if (inj->swRetries_attempted <= inj->swRetries) {
                uint64_t delay_ns = INJECTOR_BACKOFF_BASE_NS;
                if (snap_flags & INJ_FLAG_EXP_BACKOFF) {
                    uint8_t step = inj->backoff_step;
                    if (step < INJECTOR_BACKOFF_MAX_STEPS) inj->backoff_step++;
                    delay_ns = INJECTOR_BACKOFF_BASE_NS << step;
                }
                inj->next_desired_ns = timer_get_time_ns_fine() + delay_ns;
                inj->state = INJ_STATE_READY;
            } else {
                inj->tx_errors++;
                mgr->totalErrorsAllTime++;
                inj->swRetries_attempted = 0u;
                inj->backoff_step        = 0u;
                inj->active              = false;
                inj->state               = INJ_STATE_IDLE;
                INJ_LOG("inject: [%s] deactivated after %u SW retries exhausted\n",
                        inj->name, inj->swRetries);
            }
        } else {
            inj->swRetries_attempted = 0u;
            inj->backoff_step        = 0u;
            inj->stat_bursts_ok++;

            bool done = (maxPkts > 0u && inj->packets_sent >= maxPkts)
                     || (interval_ns == 0u);
            if (done) {
                inj->active = false;
                inj->state  = INJ_STATE_DONE;
            } else {
                inj->burst_idx      += burst_ok;
                inj->next_desired_ns = inj->start_ns
                                     + inj->burst_idx * interval_ns;
                uint64_t tnow = timer_get_time_ns_fine();
                if (inj->next_desired_ns <= tnow) {
                    uint64_t elapsed     = tnow - inj->start_ns;
                    uint64_t new_idx     = elapsed / interval_ns + 1u;
                    inj->burst_idx       = new_idx;
                    inj->next_desired_ns = inj->start_ns + new_idx * interval_ns;
                }
                inj->state = INJ_STATE_READY;
            }
        }
        xSemaphoreGive(mgr->lock);
    }

    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    mgr->scheduler_task = NULL;
    mgr->sched_state    = INJ_SCHED_STOPPED;
    xSemaphoreGive(mgr->lock);
    xSemaphoreGive(mgr->stop_done_sem);
    vTaskDelete(NULL);
}

/* =========================================================================
 * Deterministic stop — state machine, no force-delete
 * ========================================================================= */
static int stop_scheduler_internal(injectorManager *mgr)
{
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    inj_sched_state_t cur = mgr->sched_state;
    if (cur == INJ_SCHED_IDLE || cur == INJ_SCHED_STOPPED) {
        xSemaphoreGive(mgr->lock);
        return INJ_OK;
    }
    if (cur == INJ_SCHED_STUCK) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_STATE;
    }

    mgr->sched_state = INJ_SCHED_STOPPING;
    xSemaphoreGive(mgr->lock);
    xSemaphoreGive(mgr->wake_sem);

    if (xSemaphoreTake(mgr->stop_done_sem,
                        pdMS_TO_TICKS(INJECTOR_STOP_TIMEOUT_MS)) == pdTRUE) {
        return INJ_OK;
    }

    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    mgr->sched_state = INJ_SCHED_STUCK;
    xSemaphoreGive(mgr->lock);
    INJ_LOG("inject: BUG — scheduler task stuck (timeout %u ms). "
            "Call resetStuck() before destroy().\n", INJECTOR_STOP_TIMEOUT_MS);
    return INJ_ERR;
}

/* =========================================================================
 * Lifecycle
 * ========================================================================= */
injectorManager *injectorManager_create(void)
{
    if (!timer_is_ready()) {
        INJ_LOG("inject: call timer_init() first\n"); return NULL;
    }
    injectorManager *mgr = pvPortMalloc(sizeof(injectorManager));
    if (!mgr) return NULL;
    memset(mgr, 0, sizeof(injectorManager));
    mgr->lock          = xSemaphoreCreateMutex();
    mgr->wake_sem      = xSemaphoreCreateBinary();
    mgr->stop_done_sem = xSemaphoreCreateBinary();
    if (!mgr->lock || !mgr->wake_sem || !mgr->stop_done_sem) {
        if (mgr->lock)          vSemaphoreDelete(mgr->lock);
        if (mgr->wake_sem)      vSemaphoreDelete(mgr->wake_sem);
        if (mgr->stop_done_sem) vSemaphoreDelete(mgr->stop_done_sem);
        vPortFree(mgr); return NULL;
    }
    mgr->current_tx_power = -1;
    mgr->current_channel  = 0u;
    mgr->wlan_idx         = STA_WLAN_INDEX;
    mgr->sched_state      = INJ_SCHED_IDLE;
    return mgr;
}

void injectorManager_destroy(injectorManager *mgr)
{
    if (!mgr) return;
    int rc = stop_scheduler_internal(mgr);
    if (rc != INJ_OK) {
        INJ_LOG("inject: destroy aborted — scheduler stuck; "
                "call resetStuck() then retry destroy()\n");
        return;
    }
    injectorManager_clearAll(mgr);
    vSemaphoreDelete(mgr->lock);
    vSemaphoreDelete(mgr->wake_sem);
    vSemaphoreDelete(mgr->stop_done_sem);
    vPortFree(mgr);
}

void injectorManager_setWlanIndex(injectorManager *mgr, uint8_t idx) {
    if (!mgr) return;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    mgr->wlan_idx = idx;
    xSemaphoreGive(mgr->lock);
}

void injectorManager_clearAll(injectorManager *mgr) {
    if (!mgr) return;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    for (int i = 0; i < (int)INJECTOR_MAX; i++)
        if (mgr->injectors[i].in_use)
            delete_injector(mgr, &mgr->injectors[i]);
    xSemaphoreGive(mgr->lock);
}

/* =========================================================================
 * setInjectorEx
 * ========================================================================= */
int injectorManager_setInjectorEx(injectorManager *mgr,
                                   const char      *name,
                                   const uint8_t   *packetData,
                                   uint32_t         packetLen,
                                   uint8_t          channel,
                                   uint64_t         start_time_ns,
                                   uint64_t         interval_ns,
                                   uint32_t         burst_count,
                                   uint32_t         maxPackets,
                                   uint8_t          hwRetries,
                                   uint8_t          swRetries,
                                   inject_rate_t    rate,
                                   int8_t           tx_power_dbm,
                                   inject_flags_t   flags,
                                   inject_ac_t      ac_queue,
                                   uint8_t          priority)
{
    if (!mgr || !name || !packetData || packetLen == 0u) return INJ_ERR_INVALID_ARG;
    if (strlen(name) >= INJECTOR_NAME_MAX)               return INJ_ERR_INVALID_ARG;
    if (packetLen > INJECTOR_MAX_PACKET_SIZE)            return INJ_ERR_INVALID_ARG;
    if (!is_valid_channel(channel))                      return INJ_ERR_CHANNEL;
    if (!is_valid_rate(rate))                            return INJ_ERR_RATE;
    if (!timer_is_ready())                               return INJ_ERR_TIMER;

    uint8_t *copy = pvPortMalloc(packetLen);
    if (!copy) return INJ_ERR_NO_SPACE;
    memcpy(copy, packetData, packetLen);

    uint64_t now   = timer_get_time_ns_fine();
    uint64_t start = (start_time_ns == 0u || start_time_ns <= now)
                   ? now : start_time_ns;

    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *existing = find_by_name(mgr, name);
    PacketInjector *inj      = existing ? existing : find_free_slot(mgr);
    if (!inj) { vPortFree(copy); xSemaphoreGive(mgr->lock); return INJ_ERR_NO_SPACE; }

    inj->generation++;
    if (inj->packetData) { vPortFree(inj->packetData); inj->packetData = NULL; }
    inj->packetData = copy;
    inj->packetLen  = packetLen;
    configASSERT(inj->packetData != NULL);

    if (!existing) {
        strncpy(inj->name, name, INJECTOR_NAME_MAX - 1u);
        inj->name[INJECTOR_NAME_MAX - 1u] = '\0';
        mgr->injectorCount++;
    }

    inj->packets_sent = 0u; inj->seq_num = 0u;
    inj->tx_errors    = 0u; inj->swRetries_attempted = 0u;
    inj->swRetries_total = 0u; inj->backoff_step = 0u;
    inj->total_air_time_us = 0ULL;
    inj->stat_init   = false;
    inj->stat_mean_ns   = 0;
    inj->stat_var_ns2   = 0ULL;
    inj->stat_stddev_ns = 0ULL;
    inj->stat_min_ns = INT64_MAX; inj->stat_max_ns = INT64_MIN;
    inj->stat_misses = 0u; inj->stat_skips = 0u; inj->stat_bursts_ok = 0u;
    memset(inj->stat_histogram, 0, sizeof(inj->stat_histogram));

    inj->in_use          = true;
    inj->active          = false;
    inj->state           = INJ_STATE_IDLE;
    inj->channel         = channel;
    inj->interval_ns     = interval_ns;
    inj->burst_count     = (burst_count < 1u) ? 1u : burst_count;
    inj->maxPackets      = maxPackets;
    inj->hwRetries       = hwRetries;
    inj->swRetries       = swRetries;
    inj->tx_power_dbm    = tx_power_dbm;
    inj->tx_rate         = rate;
    inj->flags           = flags;
    inj->ac_queue        = ac_queue;
    inj->priority        = priority;
    inj->start_ns        = start;
    inj->burst_idx       = 0u;
    inj->next_desired_ns = start;
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

int injectorManager_deleteInjector(injectorManager *mgr, const char *name) {
    if (!mgr || !name) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = find_by_name(mgr, name);
    if (!inj) { xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }
    delete_injector(mgr, inj);
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

int injectorManager_activateInjector(injectorManager *mgr, const char *name) {
    if (!mgr || !name) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = find_by_name(mgr, name);
    if (!inj) { xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }
    configASSERT(inj->packetData != NULL);
    uint64_t now = timer_get_time_ns_fine();
    if (inj->start_ns < now) { inj->start_ns = now; inj->burst_idx = 0u; }
    inj->next_desired_ns     = inj->start_ns;
    inj->active              = true;
    inj->state               = INJ_STATE_READY;
    inj->swRetries_attempted = 0u;
    inj->backoff_step        = 0u;
    xSemaphoreGive(mgr->lock);
    xSemaphoreGive(mgr->wake_sem);
    return INJ_OK;
}

int injectorManager_deactivateInjector(injectorManager *mgr, const char *name) {
    if (!mgr || !name) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = find_by_name(mgr, name);
    if (!inj) { xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }
    inj->active = false; inj->state = INJ_STATE_IDLE;
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

/* =========================================================================
 * Dynamic parameter update macro
 * ========================================================================= */
#define SET_FIELD(field, val, bump_gen)                                    \
    do {                                                                   \
        if (!mgr || !name) return INJ_ERR_INVALID_ARG;                     \
        xSemaphoreTake(mgr->lock, portMAX_DELAY);                          \
        PacketInjector *_i = find_by_name(mgr, name);                      \
        if (!_i) { xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }  \
        _i->field = (val);                                                 \
        if (bump_gen) _i->generation++;                                    \
        xSemaphoreGive(mgr->lock);                                         \
        xSemaphoreGive(mgr->wake_sem);                                     \
        return INJ_OK;                                                     \
    } while (0)

int injectorManager_setRate(injectorManager *mgr, const char *name, inject_rate_t r)
{ if (!is_valid_rate(r)) return INJ_ERR_RATE; SET_FIELD(tx_rate, r, true); }
int injectorManager_setChannel(injectorManager *mgr, const char *name, uint8_t ch)
{ if (!is_valid_channel(ch)) return INJ_ERR_CHANNEL; SET_FIELD(channel, ch, true); }
int injectorManager_setTxPower(injectorManager *mgr, const char *name, int8_t dbm)
{ SET_FIELD(tx_power_dbm, dbm, true); }
int injectorManager_setIntervalNs(injectorManager *mgr, const char *name, uint64_t iv)
{ SET_FIELD(interval_ns, iv, true); }
int injectorManager_setBurstCount(injectorManager *mgr, const char *name, uint32_t bc)
{ uint32_t v=(bc<1u)?1u:bc; SET_FIELD(burst_count, v, true); }
int injectorManager_setMaxPackets(injectorManager *mgr, const char *name, uint32_t mp)
{ SET_FIELD(maxPackets, mp, true); }
int injectorManager_setHwRetries(injectorManager *mgr, const char *name, uint8_t hw)
{ SET_FIELD(hwRetries, hw, true); }
int injectorManager_setSwRetries(injectorManager *mgr, const char *name, uint8_t sw)
{ SET_FIELD(swRetries, sw, true); }
int injectorManager_setFlags(injectorManager *mgr, const char *name, inject_flags_t fl)
{ SET_FIELD(flags, fl, true); }
int injectorManager_setAcQueue(injectorManager *mgr, const char *name, inject_ac_t ac)
{ SET_FIELD(ac_queue, ac, true); }
int injectorManager_setPriority(injectorManager *mgr, const char *name, uint8_t p)
{ SET_FIELD(priority, p, false); }

int injectorManager_setPacketData(injectorManager *mgr, const char *name,
                                   const uint8_t *data, uint32_t len) {
    if (!mgr || !name || !data || len == 0u || len > INJECTOR_MAX_PACKET_SIZE)
        return INJ_ERR_INVALID_ARG;
    uint8_t *nd = pvPortMalloc(len);
    if (!nd) return INJ_ERR_NO_SPACE;
    memcpy(nd, data, len);
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = find_by_name(mgr, name);
    if (!inj) { vPortFree(nd); xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }
    uint8_t *old = inj->packetData;
    inj->packetData = nd; inj->packetLen = len; inj->generation++;
    xSemaphoreGive(mgr->lock);
    if (old) vPortFree(old);
    xSemaphoreGive(mgr->wake_sem);
    return INJ_OK;
}

int injectorManager_setMutateCallback(injectorManager *mgr, const char *name,
                                       inj_mutate_cb_t cb, void *ctx) {
    if (!mgr || !name) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = find_by_name(mgr, name);
    if (!inj) { xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }
    inj->mutate_cb = cb; inj->mutate_ctx = ctx;
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

/* =========================================================================
 * Statistics
 * ========================================================================= */
int injectorManager_getInfo(injectorManager *mgr, const char *name, InjectorInfo *out) {
    if (!mgr || !name || !out) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = find_by_name(mgr, name);
    if (!inj) { xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }
    strncpy(out->name, inj->name, INJECTOR_NAME_MAX);
    out->name[INJECTOR_NAME_MAX-1u] = '\0';
    out->active=inj->active; out->channel=inj->channel; out->priority=inj->priority;
    out->interval_ns=inj->interval_ns; out->burst_count=inj->burst_count;
    out->maxPackets=inj->maxPackets; out->packets_sent=inj->packets_sent;
    out->tx_errors=inj->tx_errors; out->tx_retries=inj->swRetries_total;
    out->maxHwRetries=inj->hwRetries; out->tx_rate=inj->tx_rate;
    out->tx_power_dbm=inj->tx_power_dbm; out->flags=inj->flags;
    out->ac_queue=inj->ac_queue; out->packetLen=inj->packetLen;
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

int injectorManager_getTimingStats(injectorManager *mgr, const char *name,
                                    InjectorTimingStats *out) {
    if (!mgr || !name || !out) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = find_by_name(mgr, name);
    if (!inj) { xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }
    out->jitter_mean_ns    = inj->stat_mean_ns;
    out->jitter_var_ns2    = inj->stat_var_ns2;
    out->jitter_stddev_ns  = inj->stat_stddev_ns;
    out->jitter_min_ns     = inj->stat_init ? inj->stat_min_ns : 0;
    out->jitter_max_ns     = inj->stat_init ? inj->stat_max_ns : 0;
    out->switch_mean_ns    = inj->stat_switch_mean_ns;
    out->deadline_misses   = inj->stat_misses;
    out->deadline_skips    = inj->stat_skips;
    out->burst_completions = inj->stat_bursts_ok;
    out->tx_air_time_us    = (uint32_t)inj->total_air_time_us;
    memcpy(out->jitter_histogram, inj->stat_histogram, sizeof(inj->stat_histogram));
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

int injectorManager_listInjectors(injectorManager *mgr,
                                   char names[][INJECTOR_NAME_MAX], int max) {
    if (!mgr || !names || max <= 0) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    int cnt = 0;
    for (int i = 0; i < (int)INJECTOR_MAX && cnt < max; i++) {
        if (!mgr->injectors[i].in_use) continue;
        strncpy(names[cnt], mgr->injectors[i].name, INJECTOR_NAME_MAX);
        names[cnt][INJECTOR_NAME_MAX-1u] = '\0'; cnt++;
    }
    xSemaphoreGive(mgr->lock); return cnt;
}

uint64_t injectorManager_getTotalPackets(injectorManager *mgr) {
    if (!mgr) return 0ULL;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    uint64_t t = mgr->totalPacketsAllTime;
    xSemaphoreGive(mgr->lock); return t;
}

uint32_t injectorManager_getActiveCount(injectorManager *mgr) {
    if (!mgr) return 0u;
    uint32_t cnt = 0u;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    for (int i = 0; i < (int)INJECTOR_MAX; i++) {
        PacketInjector *inj = &mgr->injectors[i];
        if (inj->in_use && (inj->state == INJ_STATE_READY ||
                             inj->state == INJ_STATE_SENDING)) cnt++;
    }
    xSemaphoreGive(mgr->lock); return cnt;
}

uint32_t injectorManager_getTotalErrors(injectorManager *mgr) {
    if (!mgr) return 0u;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    uint32_t e = mgr->totalErrorsAllTime;
    xSemaphoreGive(mgr->lock); return e;
}

int injectorManager_resetStats(injectorManager *mgr, const char *name) {
    if (!mgr || !name) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = find_by_name(mgr, name);
    if (!inj) { xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }
    inj->packets_sent=0u; inj->tx_errors=0u;
    inj->swRetries_attempted=0u; inj->swRetries_total=0u;
    xSemaphoreGive(mgr->lock); return INJ_OK;
}

int injectorManager_resetTimingStats(injectorManager *mgr, const char *name) {
    if (!mgr || !name) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = find_by_name(mgr, name);
    if (!inj) { xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }
    inj->stat_init      = false;
    inj->stat_mean_ns   = 0;
    inj->stat_var_ns2   = 0ULL;
    inj->stat_stddev_ns = 0ULL;
    inj->stat_min_ns    = INT64_MAX;
    inj->stat_max_ns    = INT64_MIN;
    inj->stat_switch_mean_ns = 0ULL;
    inj->stat_misses    = 0u;
    inj->stat_skips     = 0u;
    inj->stat_bursts_ok = 0u;
    memset(inj->stat_histogram, 0, sizeof(inj->stat_histogram));
    xSemaphoreGive(mgr->lock); return INJ_OK;
}

inj_sched_state_t injectorManager_getSchedState(injectorManager *mgr) {
    if (!mgr) return INJ_SCHED_IDLE;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    inj_sched_state_t s = mgr->sched_state;
    xSemaphoreGive(mgr->lock); return s;
}

int injectorManager_resetStuck(injectorManager *mgr)
{
    if (!mgr) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    if (mgr->sched_state != INJ_SCHED_STUCK) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_STATE;
    }
    if (mgr->scheduler_task) {
        INJ_LOG("inject: force-deleting stuck scheduler task — "
                "re-init Wi-Fi stack before restarting injector\n");
        vTaskDelete(mgr->scheduler_task);
        mgr->scheduler_task = NULL;
    }
    mgr->sched_state = INJ_SCHED_IDLE;
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

/* =========================================================================
 * Scheduler control
 * ========================================================================= */
int injectorManager_startSchedulerTask(injectorManager *mgr, UBaseType_t prio) {
    return injectorManager_startSchedulerTaskEx(mgr, prio, INJECTOR_SCHED_STACK_WORDS);
}

int injectorManager_startSchedulerTaskEx(injectorManager *mgr,
                                          UBaseType_t prio, uint32_t stackWords) {
    if (!mgr) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    if (mgr->sched_state != INJ_SCHED_IDLE &&
        mgr->sched_state != INJ_SCHED_STOPPED) {
        xSemaphoreGive(mgr->lock); return INJ_ERR_STATE;
    }
    xSemaphoreTake(mgr->stop_done_sem, 0);
    mgr->sched_state = INJ_SCHED_RUNNING;
    BaseType_t r = xTaskCreate(schedulerTask, "inj_sched", stackWords,
                               mgr, prio, &mgr->scheduler_task);
    if (r != pdPASS) {
        mgr->sched_state    = INJ_SCHED_IDLE;
        mgr->scheduler_task = NULL;
        xSemaphoreGive(mgr->lock); return INJ_ERR;
    }
    xSemaphoreGive(mgr->lock); return INJ_OK;
}

int injectorManager_stopSchedulerTask(injectorManager *mgr) {
    if (!mgr) return INJ_ERR_INVALID_ARG;
    configASSERT(xTaskGetCurrentTaskHandle() != mgr->scheduler_task);
    if (xTaskGetCurrentTaskHandle() == mgr->scheduler_task) return INJ_ERR_STATE;
    return stop_scheduler_internal(mgr);
}
