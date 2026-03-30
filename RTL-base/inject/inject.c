/**
 * @file inject.c
 * @brief Wi-Fi Packet Injector Manager – high-precision scheduling
 *        with latency compensation and multi-task safety.
 */

#include "inject.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "wifi_api.h"
#include "wifi_api_types.h"
#include "wifi_api_ext.h"

/*============================================================================
 * Logging
 *============================================================================*/
static inj_log_cb_t log_cb = NULL;

void injectorManager_setLogCallback(inj_log_cb_t cb) {
    log_cb = cb;
}

#define INJ_LOG(fmt, ...)                                               \
    do {                                                                \
        if (log_cb) {                                                   \
            char _buf[256];                                             \
            snprintf(_buf, sizeof(_buf), fmt, ##__VA_ARGS__);           \
            log_cb(_buf);                                               \
        } else {                                                        \
            printf(fmt, ##__VA_ARGS__);                                 \
        }                                                               \
    } while (0)

/*============================================================================
 * Internal state machine
 *============================================================================*/
typedef enum {
    INJ_STATE_IDLE    = 0,
    INJ_STATE_READY,
    INJ_STATE_SENDING,
    INJ_STATE_DONE
} injector_state_t;

typedef struct {
    char             name[INJECTOR_NAME_MAX];
    bool             in_use;
    bool             active;
    uint8_t          channel;
    uint64_t         interval_ns;
    uint64_t         next_desired_ns;
    uint32_t         maxPackets;
    uint32_t         packets_sent;
    uint32_t         tx_errors;
    uint32_t         swRetries_total;
    uint8_t          hwRetries;
    uint8_t          swRetries;
    uint8_t          swRetries_attempted;
    int8_t           tx_power_dbm;
    inject_rate_t    tx_rate;
    inject_flags_t   flags;
    inject_ac_t      ac_queue;
    uint8_t         *packetData;
    uint32_t         packetLen;
    injector_state_t state;
    uint32_t         generation;

    bool     ema_sched_init;
    bool     ema_switch_init;
    int64_t  last_sched_delay_ns;
    int64_t  avg_sched_delay_ns;
    uint64_t last_switch_delay_ns;
    uint64_t avg_switch_delay_ns;
} PacketInjector;

struct injectorManager {
    PacketInjector    injectors[INJECTOR_MAX];
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
    volatile bool     scheduler_running;
    uint8_t (*power_map_cb)(int8_t dbm);
};

static uint32_t timer_freq_hz     = 0;
static uint32_t last_timer_count  = 0;
static uint64_t time_offset_ns    = 0;
static bool     timer_initialized = false;

extern void     init_highres_timer(void);
extern uint32_t read_highres_timer(void);

#if defined(portMUX_TYPE)
static portMUX_TYPE s_timer_mux = portMUX_INITIALIZER_UNLOCKED;
#  define TIMER_CRITICAL_ENTER()  portENTER_CRITICAL(&s_timer_mux)
#  define TIMER_CRITICAL_EXIT()   portEXIT_CRITICAL(&s_timer_mux)
#else
#  define TIMER_CRITICAL_ENTER()  taskENTER_CRITICAL()
#  define TIMER_CRITICAL_EXIT()   taskEXIT_CRITICAL()
#endif

void injector_set_timer_freq_hz(uint32_t freq) {
    timer_freq_hz = freq;
}

static void init_timer_once(void) {
    TIMER_CRITICAL_ENTER();
    if (timer_initialized) { TIMER_CRITICAL_EXIT(); return; }
    if (timer_freq_hz == 0) {
        TIMER_CRITICAL_EXIT();
        INJ_LOG("ERROR: timer frequency not set. Call injector_set_timer_freq_hz() first.\n");
        return;
    }
    init_highres_timer();
    last_timer_count  = read_highres_timer();
    time_offset_ns    = 0;
    timer_initialized = true;
    TIMER_CRITICAL_EXIT();
}

uint64_t platform_get_time_ns(void) {
    if (!timer_initialized) {
        init_timer_once();
        if (!timer_initialized) return 0;
    }
    TIMER_CRITICAL_ENTER();
    uint32_t current = read_highres_timer();
    uint32_t delta   = (current >= last_timer_count)
                       ? (current - last_timer_count)
                       : (0xFFFFFFFFU - last_timer_count + current + 1U);
    time_offset_ns  += (uint64_t)delta * 1000000000ULL / timer_freq_hz;
    last_timer_count = current;
    uint64_t ret     = time_offset_ns;
    TIMER_CRITICAL_EXIT();
    return ret;
}

/*============================================================================
 * TX power mapping
 *============================================================================*/
static uint8_t default_dbm_to_percentage(int8_t dbm) {
    if (dbm <= 0)  return 13;
    if (dbm <= 5)  return 25;
    if (dbm <= 10) return 50;
    if (dbm <= 15) return 75;
    return 100;
}

void injectorManager_setPowerMappingCallback(injectorManager *mgr,
                                             uint8_t (*callback)(int8_t dbm)) {
    if (!mgr) return;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    mgr->power_map_cb = callback;
    xSemaphoreGive(mgr->lock);
}

__attribute__((weak)) int wifi_set_tx_power_percentage(uint8_t wlan_idx, uint8_t percentage) {
    (void)wlan_idx; (void)percentage; return 0;
}

static int set_tx_power_percentage(uint8_t wlan_idx, int8_t power_dbm,
                                   uint8_t (*map_cb)(int8_t)) {
    if (power_dbm < 0) return 0;
    uint8_t perc = map_cb ? map_cb(power_dbm) : default_dbm_to_percentage(power_dbm);
    return wifi_set_tx_power_percentage(wlan_idx, perc);
}

/*============================================================================
 * Internal helpers
 *============================================================================*/
static PacketInjector *findInjectorByName(injectorManager *mgr, const char *name) {
    for (int i = 0; i < INJECTOR_MAX; i++) {
        PacketInjector *inj = &mgr->injectors[i];
        if (inj->in_use && strcmp(inj->name, name) == 0)
            return inj;
    }
    return NULL;
}

static PacketInjector *findFreeSlot(injectorManager *mgr) {
    for (int i = 0; i < INJECTOR_MAX; i++)
        if (!mgr->injectors[i].in_use)
            return &mgr->injectors[i];
    return NULL;
}

static void deleteInjector(injectorManager *mgr, PacketInjector *inj) {
    if (!inj->in_use) return;
    inj->active = false;
    inj->state  = INJ_STATE_IDLE;
    if (inj->packetData) {
        vPortFree(inj->packetData);
        inj->packetData = NULL;
    }
    inj->generation++;
    inj->in_use  = false;
    inj->name[0] = '\0';
    mgr->injectorCount--;
}

/*============================================================================
 * Packet transmission
 *============================================================================*/
static int sendPacket(const uint8_t *packetData, uint32_t packetLen,
                      uint8_t wlan_idx, inject_rate_t rate,
                      uint8_t hwRetries, inject_ac_t ac_queue,
                      inject_flags_t flags) {
    struct rtw_raw_frame_desc desc = {0};
    desc.wlan_idx    = wlan_idx;
    desc.buf         = (uint8_t *)packetData;
    desc.buf_len     = packetLen;
    desc.tx_rate     = (uint8_t)rate;
    desc.retry_limit = hwRetries;
    desc.ac_queue    = (uint8_t)ac_queue;
    desc.sgi         = (flags & INJ_FLAG_USE_SHORT_GI) ? 1 : 0;
    desc.agg_en      = (flags & INJ_FLAG_AGGREGATE)    ? 1 : 0;
    desc.no_ack      = (flags & INJ_FLAG_NO_ACK)       ? 1 : 0;
    int ret = wifi_send_raw_frame(&desc);
    if (ret != 0) INJ_LOG("sendPacket error: %d\n", ret);
    return ret;
}

/*============================================================================
 * Min-heap – earliest-deadline-first scheduling
 *============================================================================*/
typedef struct {
    PacketInjector *inj;
    uint64_t        deadline_ns;
} SchedEntry;

static void heap_swap(SchedEntry *a, SchedEntry *b) {
    SchedEntry tmp = *a; *a = *b; *b = tmp;
}

static void heap_sift_up(SchedEntry *heap, int idx) {
    while (idx > 0) {
        int parent = (idx - 1) / 2;
        if (heap[parent].deadline_ns <= heap[idx].deadline_ns) break;
        heap_swap(&heap[parent], &heap[idx]);
        idx = parent;
    }
}

static void heap_sift_down(SchedEntry *heap, int size, int idx) {
    for (;;) {
        int left = idx * 2 + 1, right = idx * 2 + 2, smallest = idx;
        if (left  < size && heap[left ].deadline_ns < heap[smallest].deadline_ns) smallest = left;
        if (right < size && heap[right].deadline_ns < heap[smallest].deadline_ns) smallest = right;
        if (smallest == idx) break;
        heap_swap(&heap[smallest], &heap[idx]);
        idx = smallest;
    }
}

static void heap_insert(SchedEntry *heap, int *size, SchedEntry entry) {
    if (*size >= INJECTOR_MAX) {
        INJ_LOG("BUG: heap_insert overflow (size=%d)\n", *size);
        return;
    }
    heap[*size] = entry;
    heap_sift_up(heap, *size);
    (*size)++;
}

static SchedEntry heap_pop_min(SchedEntry *heap, int *size) {
    SchedEntry min = heap[0];
    (*size)--;
    heap[0] = heap[*size];
    heap_sift_down(heap, *size, 0);
    return min;
}

static void heap_clear(SchedEntry *heap, int *size) { *size = 0; }

static void schedulerTask(void *param) {
    injectorManager *mgr = (injectorManager *)param;
    SchedEntry heap[INJECTOR_MAX];
    int heap_size = 0;

    for (;;) {
        xSemaphoreTake(mgr->lock, portMAX_DELAY);
        bool running = mgr->scheduler_running;
        xSemaphoreGive(mgr->lock);
        if (!running) break;

        uint64_t now = platform_get_time_ns();

        xSemaphoreTake(mgr->lock, portMAX_DELAY);
        heap_clear(heap, &heap_size);
        for (int i = 0; i < INJECTOR_MAX; i++) {
            PacketInjector *inj = &mgr->injectors[i];
            if (inj->in_use && inj->state == INJ_STATE_READY) {
                SchedEntry e = { .inj = inj, .deadline_ns = inj->next_desired_ns };
                heap_insert(heap, &heap_size, e);
            }
        }
        xSemaphoreGive(mgr->lock);

        if (heap_size == 0) {
            xSemaphoreTake(mgr->wake_sem, portMAX_DELAY);
            continue;
        }

        uint64_t deadline = heap[0].deadline_ns;
        if (deadline > now) {
            uint64_t diff_ns = deadline - now;
            if (diff_ns > INJECTOR_SPIN_GUARD_NS) {
                uint64_t sleep_ns = diff_ns - INJECTOR_SPIN_GUARD_NS;
                TickType_t ticks  = pdMS_TO_TICKS(sleep_ns / 1000000ULL);
                if (ticks > 0)
                    xSemaphoreTake(mgr->wake_sem, ticks);
            } else {
                taskYIELD();
            }
            continue;
        }

        SchedEntry earliest  = heap_pop_min(heap, &heap_size);
        PacketInjector *inj  = earliest.inj;

        int64_t sched_error = (int64_t)(now - earliest.deadline_ns);

        xSemaphoreTake(mgr->lock, portMAX_DELAY);
        if (!inj->in_use || inj->state != INJ_STATE_READY) {
            xSemaphoreGive(mgr->lock);
            continue;
        }
        uint32_t packet_len = inj->packetLen;
        uint32_t gen        = inj->generation;
        xSemaphoreGive(mgr->lock);

        uint8_t *packet_copy = pvPortMalloc(packet_len);
        if (!packet_copy) {
            INJ_LOG("schedulerTask: OOM for %u bytes – skipping cycle\n", (unsigned)packet_len);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        xSemaphoreTake(mgr->lock, portMAX_DELAY);
        if (!inj->in_use || inj->generation != gen || inj->state != INJ_STATE_READY) {
            vPortFree(packet_copy);
            xSemaphoreGive(mgr->lock);
            continue;
        }

        inj->last_sched_delay_ns = sched_error;
        if (!inj->ema_sched_init) {
            inj->avg_sched_delay_ns = sched_error;
            inj->ema_sched_init     = true;
        } else {
            inj->avg_sched_delay_ns = (inj->avg_sched_delay_ns * 7 + sched_error) / 8;
        }

        memcpy(packet_copy, inj->packetData, inj->packetLen);

        char name_copy[INJECTOR_NAME_MAX];
        strncpy(name_copy, inj->name, sizeof(name_copy));
        name_copy[sizeof(name_copy) - 1] = '\0';

        uint64_t       interval_ns  = inj->interval_ns;
        uint32_t       maxPackets   = inj->maxPackets;
        uint8_t        swRetries    = inj->swRetries;
        inject_rate_t  rate         = inj->tx_rate;
        uint8_t        hwRetries    = inj->hwRetries;
        inject_ac_t    ac_queue     = inj->ac_queue;
        inject_flags_t flags        = inj->flags;
        int8_t         tx_power_dbm = inj->tx_power_dbm;
        uint8_t        channel      = inj->channel;

        uint8_t  wlan_idx              = mgr->wlan_idx;
        uint8_t (*power_map_cb)(int8_t) = mgr->power_map_cb;
        uint8_t  current_channel       = mgr->current_channel;
        int8_t   current_tx_power      = mgr->current_tx_power;

        inj->state = INJ_STATE_SENDING;
        xSemaphoreGive(mgr->lock);

        uint64_t t_start = platform_get_time_ns();
        int ret = 0;

        if (channel != 0 && channel != current_channel) {
            if (wifi_set_channel(wlan_idx, channel) == 0) {
                xSemaphoreTake(mgr->lock, portMAX_DELAY);
                mgr->current_channel = channel;
                xSemaphoreGive(mgr->lock);
            } else {
                ret = -1;
                INJ_LOG("Failed to set channel %d for '%s'\n", channel, name_copy);
            }
        }
        if (ret == 0 && tx_power_dbm >= 0 && tx_power_dbm != current_tx_power) {
            if (set_tx_power_percentage(wlan_idx, tx_power_dbm, power_map_cb) == 0) {
                xSemaphoreTake(mgr->lock, portMAX_DELAY);
                mgr->current_tx_power = tx_power_dbm;
                xSemaphoreGive(mgr->lock);
            } else {
                ret = -1;
                INJ_LOG("Failed to set TX power %d dBm for '%s'\n", tx_power_dbm, name_copy);
            }
        }
        if (ret == 0)
            ret = sendPacket(packet_copy, packet_len, wlan_idx,
                             rate, hwRetries, ac_queue, flags);

        uint64_t t_end        = platform_get_time_ns();
        uint64_t switch_delay = t_end - t_start;
        vPortFree(packet_copy);

        xSemaphoreTake(mgr->lock, portMAX_DELAY);

        inj->last_switch_delay_ns = switch_delay;
        if (!inj->ema_switch_init) {
            inj->avg_switch_delay_ns = switch_delay;
            inj->ema_switch_init     = true;
        } else {
            inj->avg_switch_delay_ns = (inj->avg_switch_delay_ns * 7 + switch_delay) / 8;
        }

        if (!inj->in_use || inj->generation != gen || inj->state != INJ_STATE_SENDING) {
            if (inj->in_use && inj->state == INJ_STATE_SENDING)
                inj->state = INJ_STATE_READY;
            xSemaphoreGive(mgr->lock);
            continue;
        }

        if (ret == 0) {
            inj->packets_sent++;
            mgr->totalPacketsAllTime++;
            inj->swRetries_attempted = 0;

            bool done = (interval_ns == 0) ||
                        (maxPackets > 0 && inj->packets_sent >= maxPackets);

            if (done) {
                inj->state  = INJ_STATE_DONE;
                inj->active = false;
            } else {
                int64_t compensation = inj->avg_sched_delay_ns +
                                       (int64_t)inj->avg_switch_delay_ns;
                if (compensation < 0) compensation = 0;
                if ((uint64_t)compensation > interval_ns / 2)
                    compensation = (int64_t)(interval_ns / 2);

                uint64_t next = inj->next_desired_ns + interval_ns
                                - (uint64_t)compensation;

                uint64_t now2 = platform_get_time_ns();
                if (next <= now2) next = now2 + 1000ULL;

                inj->next_desired_ns = next;
                inj->state = INJ_STATE_READY;
            }
        } else {
            inj->swRetries_attempted++;
            inj->swRetries_total++;

            if (inj->swRetries_attempted <= swRetries) {
                inj->next_desired_ns = platform_get_time_ns() + 1000000ULL;
                inj->state = INJ_STATE_READY;
            } else {
                inj->tx_errors++;
                mgr->totalErrorsAllTime++;
                inj->swRetries_attempted = 0;

                bool done = (interval_ns == 0) ||
                            (maxPackets > 0 && inj->packets_sent >= maxPackets);
                if (done) {
                    inj->state  = INJ_STATE_DONE;
                    inj->active = false;
                } else {
                    inj->state  = INJ_STATE_IDLE;
                    inj->active = false;
                    INJ_LOG("Injector '%s' deactivated after %u consecutive failures\n",
                            name_copy, (unsigned)(swRetries + 1));
                }
            }
        }
        xSemaphoreGive(mgr->lock);
    }

    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    mgr->scheduler_task = NULL;
    xSemaphoreGive(mgr->lock);
    xSemaphoreGive(mgr->stop_done_sem);
    vTaskDelete(NULL);
}

static void stopSchedulerInternal(injectorManager *mgr) {
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    bool was_running       = mgr->scheduler_running;
    mgr->scheduler_running = false;
    TaskHandle_t task      = mgr->scheduler_task;
    xSemaphoreGive(mgr->lock);

    if (!was_running || !task) return;

    xSemaphoreGive(mgr->wake_sem);
    
    if (xSemaphoreTake(mgr->stop_done_sem,
                       pdMS_TO_TICKS(INJECTOR_STOP_TIMEOUT_MS)) != pdTRUE) {
        INJ_LOG("WARNING: scheduler did not exit within %d ms – force deleting\n",
                INJECTOR_STOP_TIMEOUT_MS);
        xSemaphoreTake(mgr->lock, portMAX_DELAY);
        if (mgr->scheduler_task) {
            vTaskDelete(mgr->scheduler_task);
            mgr->scheduler_task = NULL;
        }
        xSemaphoreGive(mgr->lock);
    }
}

/*============================================================================
 * Public API – lifecycle
 *============================================================================*/
injectorManager *injectorManager_create(void) {
    if (timer_freq_hz == 0) {
        INJ_LOG("ERROR: call injector_set_timer_freq_hz() before injectorManager_create().\n");
        return NULL;
    }
    init_timer_once();
    if (!timer_initialized) return NULL;

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
        vPortFree(mgr);
        return NULL;
    }

    mgr->current_tx_power  = -1;
    mgr->current_channel   = 0;
    mgr->scheduler_running = false;
    mgr->wlan_idx          = STA_WLAN_INDEX;
    mgr->power_map_cb      = NULL;
    return mgr;
}

void injectorManager_destroy(injectorManager *mgr) {
    if (!mgr) return;
    stopSchedulerInternal(mgr);
    injectorManager_clearAll(mgr);
    vSemaphoreDelete(mgr->lock);
    vSemaphoreDelete(mgr->wake_sem);
    vSemaphoreDelete(mgr->stop_done_sem);
    vPortFree(mgr);
}

void injectorManager_setWlanIndex(injectorManager *mgr, uint8_t wlan_idx) {
    if (!mgr) return;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    mgr->wlan_idx = wlan_idx;
    xSemaphoreGive(mgr->lock);
}

void injectorManager_clearAll(injectorManager *mgr) {
    if (!mgr) return;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    for (int i = 0; i < INJECTOR_MAX; i++)
        if (mgr->injectors[i].in_use)
            deleteInjector(mgr, &mgr->injectors[i]);
    mgr->injectorCount = 0;
    xSemaphoreGive(mgr->lock);
}

/*============================================================================
 * Public API – injector management
 *============================================================================*/
int injectorManager_setInjectorEx(injectorManager *mgr,
                                  const char *name,
                                  const uint8_t *packetData,
                                  uint32_t packetLen,
                                  uint8_t channel,
                                  uint64_t start_time_ns,
                                  uint64_t interval_ns,
                                  uint32_t maxPackets,
                                  uint8_t hwRetries,
                                  uint8_t swRetries,
                                  inject_rate_t rate,
                                  int8_t tx_power_dbm,
                                  inject_flags_t flags,
                                  inject_ac_t ac_queue) {
    if (!mgr || !name || !packetData || packetLen == 0)
        return INJ_ERR_INVALID_ARG;
    if (strlen(name) >= INJECTOR_NAME_MAX)
        return INJ_ERR_INVALID_ARG;
    if (packetLen > INJECTOR_MAX_PACKET_SIZE)
        return INJ_ERR_INVALID_ARG;

    if (!timer_initialized) {
        init_timer_once();
        if (!timer_initialized) return INJ_ERR_TIMER;
    }
    uint64_t now   = platform_get_time_ns();
    if (now == 0)  return INJ_ERR_TIMER;
    uint64_t start = (start_time_ns == 0 || start_time_ns <= now) ? now : start_time_ns;

    uint8_t *copy = pvPortMalloc(packetLen);
    if (!copy) return INJ_ERR_NO_SPACE;
    memcpy(copy, packetData, packetLen);

    xSemaphoreTake(mgr->lock, portMAX_DELAY);

    PacketInjector *existing = findInjectorByName(mgr, name);
    PacketInjector *inj      = existing ? existing : findFreeSlot(mgr);
    if (!inj) {
        vPortFree(copy);
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_NO_SPACE;
    }

    if (inj->packetData) vPortFree(inj->packetData);

    strncpy(inj->name, name, INJECTOR_NAME_MAX - 1);
    inj->name[INJECTOR_NAME_MAX - 1] = '\0';
    inj->in_use          = true;
    inj->active          = false;
    inj->state           = INJ_STATE_IDLE;
    inj->channel         = channel;
    inj->interval_ns     = interval_ns;
    inj->maxPackets      = maxPackets;
    inj->hwRetries       = hwRetries;
    inj->swRetries       = swRetries;
    inj->tx_power_dbm    = tx_power_dbm;
    inj->tx_rate         = rate;
    inj->flags           = flags;
    inj->ac_queue        = ac_queue;
    inj->packetData      = copy;
    inj->packetLen       = packetLen;
    inj->next_desired_ns = start;
    inj->generation++;

    inj->packets_sent         = 0;
    inj->tx_errors            = 0;
    inj->swRetries_attempted  = 0;
    inj->swRetries_total      = 0;
    inj->ema_sched_init       = false;
    inj->ema_switch_init      = false;
    inj->last_sched_delay_ns  = 0;
    inj->avg_sched_delay_ns   = 0;
    inj->last_switch_delay_ns = 0;
    inj->avg_switch_delay_ns  = 0;

    if (!existing) mgr->injectorCount++;

    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

int injectorManager_deleteInjector(injectorManager *mgr, const char *name) {
    if (!mgr || !name) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = findInjectorByName(mgr, name);
    if (!inj) { xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }
    deleteInjector(mgr, inj);
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

int injectorManager_activateInjector(injectorManager *mgr, const char *name) {
    if (!mgr || !name) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = findInjectorByName(mgr, name);
    if (!inj) { xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }
    inj->active              = true;
    inj->state               = INJ_STATE_READY;
    inj->swRetries_attempted = 0;
    uint64_t now = platform_get_time_ns();
    if (inj->next_desired_ns < now) inj->next_desired_ns = now;
    xSemaphoreGive(mgr->lock);
    xSemaphoreGive(mgr->wake_sem);
    return INJ_OK;
}

int injectorManager_deactivateInjector(injectorManager *mgr, const char *name) {
    if (!mgr || !name) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = findInjectorByName(mgr, name);
    if (!inj) { xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }
    inj->active = false;
    inj->state  = INJ_STATE_IDLE;
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

#define SET_FIELD(field, value, update_gen)                                    \
    do {                                                                       \
        if (!mgr || !name) return INJ_ERR_INVALID_ARG;                         \
        xSemaphoreTake(mgr->lock, portMAX_DELAY);                              \
        PacketInjector *_inj = findInjectorByName(mgr, name);                  \
        if (!_inj) { xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }   \
        uint32_t _saved_gen = _inj->generation;                                \
        while (_inj->state == INJ_STATE_SENDING) {                             \
            xSemaphoreGive(mgr->lock);                                         \
            vTaskDelay(pdMS_TO_TICKS(1));                                      \
            xSemaphoreTake(mgr->lock, portMAX_DELAY);                          \
            _inj = findInjectorByName(mgr, name);                              \
            if (!_inj || _inj->generation != _saved_gen) {                     \
                xSemaphoreGive(mgr->lock);                                     \
                return INJ_ERR_NOT_FOUND;                                      \
            }                                                                  \
        }                                                                      \
        _inj->field = (value);                                                 \
        if (update_gen) _inj->generation++;                                    \
        xSemaphoreGive(mgr->lock);                                             \
        return INJ_OK;                                                         \
    } while (0)

int injectorManager_setRate(injectorManager *mgr, const char *name, inject_rate_t rate) {
    SET_FIELD(tx_rate, rate, true);
}
int injectorManager_setChannel(injectorManager *mgr, const char *name, uint8_t channel) {
    SET_FIELD(channel, channel, true);
}
int injectorManager_setTxPower(injectorManager *mgr, const char *name, int8_t tx_power_dbm) {
    SET_FIELD(tx_power_dbm, tx_power_dbm, true);
}
int injectorManager_setIntervalNs(injectorManager *mgr, const char *name, uint64_t interval_ns) {
    SET_FIELD(interval_ns, interval_ns, true);
}
int injectorManager_setMaxPackets(injectorManager *mgr, const char *name, uint32_t maxPackets) {
    SET_FIELD(maxPackets, maxPackets, true);
}
int injectorManager_setHwRetries(injectorManager *mgr, const char *name, uint8_t hwRetries) {
    SET_FIELD(hwRetries, hwRetries, true);
}
int injectorManager_setSwRetries(injectorManager *mgr, const char *name, uint8_t swRetries) {
    SET_FIELD(swRetries, swRetries, true);
}
int injectorManager_setFlags(injectorManager *mgr, const char *name, inject_flags_t flags) {
    SET_FIELD(flags, flags, true);
}
int injectorManager_setAcQueue(injectorManager *mgr, const char *name, inject_ac_t ac_queue) {
    SET_FIELD(ac_queue, ac_queue, true);
}

int injectorManager_setPacketData(injectorManager *mgr, const char *name,
                                  const uint8_t *newData, uint32_t newLen) {
    if (!mgr || !name || !newData || newLen == 0) return INJ_ERR_INVALID_ARG;
    if (newLen > INJECTOR_MAX_PACKET_SIZE)         return INJ_ERR_INVALID_ARG;

    uint8_t *copy = pvPortMalloc(newLen);
    if (!copy) return INJ_ERR_NO_SPACE;
    memcpy(copy, newData, newLen);

    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = findInjectorByName(mgr, name);
    if (!inj) { vPortFree(copy); xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }

    uint32_t saved_gen = inj->generation;
    while (inj->state == INJ_STATE_SENDING) {
        xSemaphoreGive(mgr->lock);
        vTaskDelay(pdMS_TO_TICKS(1));
        xSemaphoreTake(mgr->lock, portMAX_DELAY);
        inj = findInjectorByName(mgr, name);
        if (!inj || inj->generation != saved_gen) {
            vPortFree(copy);
            xSemaphoreGive(mgr->lock);
            return INJ_ERR_NOT_FOUND;
        }
    }
    if (inj->packetData) vPortFree(inj->packetData);
    inj->packetData = copy;
    inj->packetLen  = newLen;
    inj->generation++;
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

/*============================================================================
 * Public API – statistics and information
 *============================================================================*/
int injectorManager_resetStats(injectorManager *mgr, const char *name) {
    if (!mgr || !name) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = findInjectorByName(mgr, name);
    if (!inj) { xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }
    inj->packets_sent        = 0;
    inj->tx_errors           = 0;
    inj->swRetries_attempted = 0;
    inj->swRetries_total     = 0;
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

int injectorManager_getInfo(injectorManager *mgr, const char *name, InjectorInfo *info) {
    if (!mgr || !name || !info) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = findInjectorByName(mgr, name);
    if (!inj) { xSemaphoreGive(mgr->lock); return INJ_ERR_NOT_FOUND; }
    strncpy(info->name, inj->name, INJECTOR_NAME_MAX);
    info->name[INJECTOR_NAME_MAX - 1] = '\0';
    info->active       = inj->active;
    info->channel      = inj->channel;
    info->interval_ns  = inj->interval_ns;
    info->maxPackets   = inj->maxPackets;
    info->packets_sent = inj->packets_sent;
    info->tx_errors    = inj->tx_errors;
    info->tx_retries   = inj->swRetries_total;
    info->maxRetries   = inj->hwRetries;
    info->tx_rate      = inj->tx_rate;
    info->tx_power_dbm = inj->tx_power_dbm;
    info->flags        = inj->flags;
    info->ac_queue     = inj->ac_queue;
    info->packetLen    = inj->packetLen;
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

int injectorManager_listInjectors(injectorManager *mgr,
                                  char names[][INJECTOR_NAME_MAX],
                                  int maxCount) {
    if (!mgr || !names || maxCount <= 0) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    int count = 0;
    for (int i = 0; i < INJECTOR_MAX && count < maxCount; i++) {
        if (mgr->injectors[i].in_use) {
            strncpy(names[count], mgr->injectors[i].name, INJECTOR_NAME_MAX);
            names[count][INJECTOR_NAME_MAX - 1] = '\0';
            count++;
        }
    }
    xSemaphoreGive(mgr->lock);
    return count;
}

uint64_t injectorManager_getTotalPackets(injectorManager *mgr) {
    if (!mgr) return 0;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    uint64_t t = mgr->totalPacketsAllTime;
    xSemaphoreGive(mgr->lock);
    return t;
}

uint32_t injectorManager_getActiveCount(injectorManager *mgr) {
    if (!mgr) return 0;
    uint32_t count = 0;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    for (int i = 0; i < INJECTOR_MAX; i++) {
        PacketInjector *inj = &mgr->injectors[i];
        if (inj->in_use &&
            (inj->state == INJ_STATE_READY || inj->state == INJ_STATE_SENDING))
            count++;
    }
    xSemaphoreGive(mgr->lock);
    return count;
}

uint32_t injectorManager_getTotalErrors(injectorManager *mgr) {
    if (!mgr) return 0;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    uint32_t e = mgr->totalErrorsAllTime;
    xSemaphoreGive(mgr->lock);
    return e;
}

/*============================================================================
 * Public API – scheduler control
 *============================================================================*/
int injectorManager_startSchedulerTask(injectorManager *mgr, UBaseType_t priority) {
    return injectorManager_startSchedulerTaskEx(mgr, priority, 2048);
}

int injectorManager_startSchedulerTaskEx(injectorManager *mgr, UBaseType_t priority,
                                         uint32_t stackSize) {
    if (!mgr) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    if (mgr->scheduler_task != NULL) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_STATE;
    }
    mgr->scheduler_running = true;
    BaseType_t ret = xTaskCreate(schedulerTask, "inj_sched", stackSize,
                                 mgr, priority, &mgr->scheduler_task);
    if (ret != pdPASS) {
        mgr->scheduler_running = false;
        mgr->scheduler_task    = NULL;
        xSemaphoreGive(mgr->lock);
        return INJ_ERR;
    }
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

int injectorManager_stopSchedulerTask(injectorManager *mgr) {
    if (!mgr) return INJ_ERR_INVALID_ARG;

    if (xTaskGetCurrentTaskHandle() == mgr->scheduler_task) {
        configASSERT(0 && "stopSchedulerTask called from scheduler task – deadlock!");
        return INJ_ERR_STATE;
    }

    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    if (!mgr->scheduler_task) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_STATE;
    }
    xSemaphoreGive(mgr->lock);

    stopSchedulerInternal(mgr);
    return INJ_OK;
}
