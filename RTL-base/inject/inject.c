/**
 * @file inject.c
 * @brief Packet injector manager – with high‑precision scheduling and flexible power mapping.
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

#define INJ_LOG(fmt, ...)                                                      \
    do {                                                                       \
        if (log_cb) {                                                          \
            char buf[256];                                                     \
            snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__);                    \
            log_cb(buf);                                                       \
        } else {                                                               \
            printf(fmt, ##__VA_ARGS__);                                        \
        }                                                                      \
    } while (0)

/*============================================================================
 * Internal data structures
 *============================================================================*/
typedef struct {
    char name[INJECTOR_NAME_MAX];
    bool in_use;
    bool active;
    uint8_t channel;
    uint64_t interval_ns;
    uint64_t next_desired_ns;
    uint32_t maxPackets;
    uint32_t packets_sent;
    uint32_t tx_errors;
    uint8_t  hwRetries;
    uint8_t  swRetries;
    uint8_t  swRetries_attempted;
    int8_t   tx_power_dbm;
    inject_rate_t tx_rate;
    inject_flags_t flags;
    inject_ac_t ac_queue;
    uint8_t *packetData;
    uint32_t packetLen;
} PacketInjector;

struct injectorManager {
    PacketInjector injectors[INJECTOR_MAX];
    uint8_t injectorCount;
    uint64_t totalPacketsAllTime;
    uint32_t totalErrorsAllTime;
    int8_t current_tx_power;
    uint8_t current_channel;
    uint8_t wlan_idx;
    SemaphoreHandle_t lock;
    SemaphoreHandle_t wake_sem;
    TaskHandle_t scheduler_task;
    volatile bool scheduler_running;
    uint8_t (*power_map_cb)(int8_t dbm);
};

/*============================================================================
 * High‑resolution timer – nanosecond precision
 *============================================================================*/
static uint32_t timer_freq_hz = 0;
static uint32_t last_timer_count = 0;
static uint64_t time_offset_ns = 0;
static bool timer_initialized = false;

extern void init_highres_timer(void);
extern uint32_t read_highres_timer(void);

void injector_set_timer_freq_hz(uint32_t freq) {
    timer_freq_hz = freq;
}

static void init_timer_once(void) {
    if (timer_initialized) return;
    init_highres_timer();
    if (timer_freq_hz == 0) {
        INJ_LOG("ERROR: timer frequency not set! Call injector_set_timer_freq_hz() first.\n");
        return;
    }
    last_timer_count = read_highres_timer();
    time_offset_ns = 0;
    timer_initialized = true;
}

uint64_t platform_get_time_ns(void) {
    if (!timer_initialized) {
        init_timer_once();
        if (!timer_initialized) return 0;
    }

    taskENTER_CRITICAL();
    uint32_t current = read_highres_timer();
    uint32_t delta;
    if (current < last_timer_count) {
        delta = (0xFFFFFFFFU - last_timer_count) + current + 1;
    } else {
        delta = current - last_timer_count;
    }
    uint64_t delta_ns = (uint64_t)delta * 1000000000ULL / timer_freq_hz;
    time_offset_ns += delta_ns;
    last_timer_count = current;
    uint64_t ret = time_offset_ns;
    taskEXIT_CRITICAL();
    return ret;
}

/*============================================================================
 * TX power mapping
 *============================================================================*/
static uint8_t default_dbm_to_percentage(int8_t dbm) {
    if (dbm <= 0)           return 13;   /* ~12.5% */
    if (dbm <= 5)           return 25;   /* 25%    */
    if (dbm <= 10)          return 50;   /* 50%    */
    if (dbm <= 15)          return 75;   /* 75%    */
    return 100;                          /* 100%   */
}

void injectorManager_setPowerMappingCallback(injectorManager *mgr, uint8_t (*callback)(int8_t dbm)) {
    if (!mgr) return;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    mgr->power_map_cb = callback;
    xSemaphoreGive(mgr->lock);
}

__attribute__((weak)) int wifi_set_tx_power_percentage(uint8_t wlan_idx, uint8_t percentage) {
    (void)wlan_idx; (void)percentage;
    return 0;
}

static int set_tx_power_percentage(uint8_t wlan_idx, int8_t power_dbm, uint8_t (*map_cb)(int8_t)) {
    if (power_dbm < 0) return 0;
    uint8_t perc;
    if (map_cb) {
        perc = map_cb(power_dbm);
    } else {
        perc = default_dbm_to_percentage(power_dbm);
    }
    return wifi_set_tx_power_percentage(wlan_idx, perc);
}

/*============================================================================
 * Helper functions
 *============================================================================*/
static PacketInjector* findInjectorByName(injectorManager *mgr, const char *name) {
    for (int i = 0; i < INJECTOR_MAX; i++) {
        PacketInjector *inj = &mgr->injectors[i];
        if (inj->in_use && strcmp(inj->name, name) == 0)
            return inj;
    }
    return NULL;
}

static PacketInjector* findFreeSlot(injectorManager *mgr) {
    for (int i = 0; i < INJECTOR_MAX; i++) {
        if (!mgr->injectors[i].in_use)
            return &mgr->injectors[i];
    }
    return NULL;
}

static void deleteInjector(injectorManager *mgr, PacketInjector *inj) {
    if (!inj->in_use) return;
    inj->active = false;
    if (inj->packetData) {
        vPortFree(inj->packetData);
        inj->packetData = NULL;
    }
    inj->in_use = false;
    inj->name[0] = '\0';
    mgr->injectorCount--;
}

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
    desc.agg_en      = (flags & INJ_FLAG_AGGREGATE) ? 1 : 0;

    int ret = wifi_send_raw_frame(&desc);
    if (ret != 0) {
        INJ_LOG("sendPacket error: %d\n", ret);
    }
    return ret;
}

/*============================================================================
 * Scheduler task – with safe packet copy and proper cleanup
 *============================================================================*/
static void schedulerTask(void *param) {
    injectorManager *mgr = (injectorManager*)param;
    uint64_t now;
    uint64_t earliest_next;
    static uint8_t packet_copy[INJECTOR_MAX_PACKET_SIZE];
    uint32_t packet_len_copy;

    for (;;) {
        if (!mgr->scheduler_running) {
            break;
        }

        now = platform_get_time_ns();

        xSemaphoreTake(mgr->lock, portMAX_DELAY);
        earliest_next = UINT64_MAX;
        for (int i = 0; i < INJECTOR_MAX; i++) {
            PacketInjector *inj = &mgr->injectors[i];
            if (!inj->in_use || !inj->active) continue;

            if (inj->maxPackets > 0 && inj->packets_sent >= inj->maxPackets) {
                deleteInjector(mgr, inj);
                continue;
            }

            if (inj->next_desired_ns > now) {
                if (inj->next_desired_ns < earliest_next)
                    earliest_next = inj->next_desired_ns;
                continue;
            }

            if (inj->packetLen > INJECTOR_MAX_PACKET_SIZE) {
                INJ_LOG("Injector %s: packet too large (%u > %d), skipping\n",
                        inj->name, inj->packetLen, INJECTOR_MAX_PACKET_SIZE);
                inj->next_desired_ns = now + 1000000ULL;
                continue;
            }
            memcpy(packet_copy, inj->packetData, inj->packetLen);
            packet_len_copy = inj->packetLen;

            char name_copy[INJECTOR_NAME_MAX];
            strncpy(name_copy, inj->name, INJECTOR_NAME_MAX);
            name_copy[INJECTOR_NAME_MAX - 1] = '\0';
            uint8_t channel = inj->channel;
            int8_t tx_power_dbm = inj->tx_power_dbm;
            uint64_t interval_ns = inj->interval_ns;
            uint32_t maxPackets = inj->maxPackets;
            uint32_t packets_sent = inj->packets_sent;
            uint8_t swRetries = inj->swRetries;
            inject_rate_t rate = inj->tx_rate;
            uint8_t hwRetries = inj->hwRetries;
            inject_ac_t ac_queue = inj->ac_queue;
            inject_flags_t flags = inj->flags;

            xSemaphoreGive(mgr->lock);

            if (channel != 0 && channel != mgr->current_channel) {
                if (wifi_set_channel(mgr->wlan_idx, channel) == 0) {
                    mgr->current_channel = channel;
                } else {
                    INJ_LOG("Failed to set channel %d for %s\n", channel, name_copy);
                    xSemaphoreTake(mgr->lock, portMAX_DELAY);
                    inj = findInjectorByName(mgr, name_copy);
                    if (inj) {
                        inj->next_desired_ns = platform_get_time_ns() + 1000000ULL;
                    }
                    xSemaphoreGive(mgr->lock);
                    continue;
                }
            }

            if (tx_power_dbm != -1 && tx_power_dbm != mgr->current_tx_power) {
                if (set_tx_power_percentage(mgr->wlan_idx, tx_power_dbm, mgr->power_map_cb) == 0) {
                    mgr->current_tx_power = tx_power_dbm;
                } else {
                    INJ_LOG("Failed to set TX power %d dBm for %s\n", tx_power_dbm, name_copy);
                    xSemaphoreTake(mgr->lock, portMAX_DELAY);
                    inj = findInjectorByName(mgr, name_copy);
                    if (inj) {
                        inj->next_desired_ns = platform_get_time_ns() + 1000000ULL;
                    }
                    xSemaphoreGive(mgr->lock);
                    continue;
                }
            }

            int ret = sendPacket(packet_copy, packet_len_copy, mgr->wlan_idx,
                                 rate, hwRetries, ac_queue, flags);

            xSemaphoreTake(mgr->lock, portMAX_DELAY);
            inj = findInjectorByName(mgr, name_copy);
            if (!inj) {
                xSemaphoreGive(mgr->lock);
                continue;
            }

            if (ret == 0) {
                inj->packets_sent++;
                mgr->totalPacketsAllTime++;
                if (interval_ns > 0) {
                    inj->next_desired_ns = now + interval_ns;
                } else {
                    inj->next_desired_ns = now + 1;
                }
                inj->swRetries_attempted = 0;
            } else {
                inj->swRetries_attempted++;
                if (inj->swRetries_attempted <= inj->swRetries) {
                    inj->next_desired_ns = now + 1000000ULL;
                } else {
                    inj->tx_errors++;
                    mgr->totalErrorsAllTime++;
                    INJ_LOG("Injector %s: send failed after %d retries\n",
                            inj->name, inj->swRetries);
                    if (inj->maxPackets > 0 && inj->packets_sent >= inj->maxPackets) {
                        deleteInjector(mgr, inj);
                    } else {
                        inj->active = false;
                    }
                    inj->swRetries_attempted = 0;
                }
            }
            xSemaphoreGive(mgr->lock);
        }
        xSemaphoreGive(mgr->lock);

        if (earliest_next != UINT64_MAX) {
            uint64_t target = earliest_next;
            while (mgr->scheduler_running && platform_get_time_ns() < target) {
                uint64_t now_local = platform_get_time_ns();
                if (now_local >= target) break;
                uint64_t remaining_ns = target - now_local;
                if (remaining_ns > 1000000ULL) {
                    injector_scheduler_wait_until_ns(now_local + 1000000ULL);
                } else {
                    injector_scheduler_wait_until_ns(target);
                    break;
                }
            }
        } else {
            TickType_t timeout = pdMS_TO_TICKS(10);
            xSemaphoreTake(mgr->wake_sem, timeout);
        }
    }

    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    mgr->scheduler_task = NULL;
    mgr->scheduler_running = false;
    xSemaphoreGive(mgr->lock);
    vTaskDelete(NULL);
}

/*============================================================================
 * Public API
 *============================================================================*/
injectorManager* injectorManager_create(void) {
    injectorManager *mgr = pvPortMalloc(sizeof(injectorManager));
    if (!mgr) return NULL;
    memset(mgr, 0, sizeof(injectorManager));
    mgr->lock = xSemaphoreCreateMutex();
    mgr->wake_sem = xSemaphoreCreateBinary();
    if (!mgr->lock || !mgr->wake_sem) {
        if (mgr->lock) vSemaphoreDelete(mgr->lock);
        if (mgr->wake_sem) vSemaphoreDelete(mgr->wake_sem);
        vPortFree(mgr);
        return NULL;
    }
    mgr->current_tx_power = -1;
    mgr->current_channel = 0;
    mgr->scheduler_running = false;
    mgr->wlan_idx = STA_WLAN_INDEX;
    mgr->power_map_cb = NULL;

    init_timer_once();

    return mgr;
}

void injectorManager_destroy(injectorManager *mgr) {
    if (!mgr) return;

    if (mgr->scheduler_running) {
        injectorManager_stopSchedulerTask(mgr);
    }

    injectorManager_clearAll(mgr);
    if (mgr->lock) vSemaphoreDelete(mgr->lock);
    if (mgr->wake_sem) vSemaphoreDelete(mgr->wake_sem);
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
    for (int i = 0; i < INJECTOR_MAX; i++) {
        PacketInjector *inj = &mgr->injectors[i];
        if (inj->in_use) deleteInjector(mgr, inj);
    }
    mgr->injectorCount = 0;
    xSemaphoreGive(mgr->lock);
}

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
    if (!mgr || !name || !packetData || packetLen == 0) return INJ_ERR_INVALID_ARG;
    if (strlen(name) >= INJECTOR_NAME_MAX) return INJ_ERR_INVALID_ARG;
    if (packetLen > INJECTOR_MAX_PACKET_SIZE) return INJ_ERR_INVALID_ARG;

    xSemaphoreTake(mgr->lock, portMAX_DELAY);

    PacketInjector *existing = findInjectorByName(mgr, name);
    if (existing) deleteInjector(mgr, existing);

    PacketInjector *inj = findFreeSlot(mgr);
    if (!inj) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_NO_SPACE;
    }

    uint8_t *copy = pvPortMalloc(packetLen);
    if (!copy) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_NO_SPACE;
    }
    memcpy(copy, packetData, packetLen);

    memset(inj, 0, sizeof(PacketInjector));
    strncpy(inj->name, name, INJECTOR_NAME_MAX - 1);
    inj->name[INJECTOR_NAME_MAX - 1] = '\0';
    inj->in_use = true;
    inj->active = false;
    inj->channel = channel;
    inj->interval_ns = interval_ns;
    inj->maxPackets = maxPackets;
    inj->hwRetries = hwRetries;
    inj->swRetries = swRetries;
    inj->swRetries_attempted = 0;
    inj->tx_power_dbm = tx_power_dbm;
    inj->tx_rate = rate;
    inj->flags = flags;
    inj->ac_queue = ac_queue;
    inj->packetData = copy;
    inj->packetLen = packetLen;

    uint64_t now = platform_get_time_ns();
    if (now == 0 && !timer_initialized) {
        vPortFree(copy);
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_TIMER;
    }
    inj->next_desired_ns = (start_time_ns == 0 || start_time_ns <= now) ? now : start_time_ns;

    mgr->injectorCount++;

    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

int injectorManager_deleteInjector(injectorManager *mgr, const char *name) {
    if (!mgr || !name) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = findInjectorByName(mgr, name);
    if (!inj) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_NOT_FOUND;
    }
    deleteInjector(mgr, inj);
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

int injectorManager_activateInjector(injectorManager *mgr, const char *name) {
    if (!mgr || !name) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = findInjectorByName(mgr, name);
    if (!inj) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_NOT_FOUND;
    }
    inj->active = true;
    inj->swRetries_attempted = 0;
    uint64_t now = platform_get_time_ns();
    if (inj->next_desired_ns < now) {
        inj->next_desired_ns = now;
    }
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

int injectorManager_deactivateInjector(injectorManager *mgr, const char *name) {
    if (!mgr || !name) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = findInjectorByName(mgr, name);
    if (!inj) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_NOT_FOUND;
    }
    inj->active = false;
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

/* Dynamic parameter updates */
#define SET_FIELD(field, value)                                                \
    do {                                                                       \
        if (!mgr || !name) return INJ_ERR_INVALID_ARG;                         \
        xSemaphoreTake(mgr->lock, portMAX_DELAY);                              \
        PacketInjector *inj = findInjectorByName(mgr, name);                   \
        if (!inj) {                                                            \
            xSemaphoreGive(mgr->lock);                                         \
            return INJ_ERR_NOT_FOUND;                                          \
        }                                                                      \
        inj->field = value;                                                    \
        xSemaphoreGive(mgr->lock);                                             \
        return INJ_OK;                                                         \
    } while(0)

int injectorManager_setRate(injectorManager *mgr, const char *name, inject_rate_t rate) { SET_FIELD(tx_rate, rate); }
int injectorManager_setChannel(injectorManager *mgr, const char *name, uint8_t channel) { SET_FIELD(channel, channel); }
int injectorManager_setTxPower(injectorManager *mgr, const char *name, int8_t tx_power_dbm) { SET_FIELD(tx_power_dbm, tx_power_dbm); }
int injectorManager_setIntervalNs(injectorManager *mgr, const char *name, uint64_t interval_ns) { SET_FIELD(interval_ns, interval_ns); }
int injectorManager_setMaxPackets(injectorManager *mgr, const char *name, uint32_t maxPackets) { SET_FIELD(maxPackets, maxPackets); }
int injectorManager_setHwRetries(injectorManager *mgr, const char *name, uint8_t hwRetries) { SET_FIELD(hwRetries, hwRetries); }
int injectorManager_setSwRetries(injectorManager *mgr, const char *name, uint8_t swRetries) { SET_FIELD(swRetries, swRetries); }
int injectorManager_setFlags(injectorManager *mgr, const char *name, inject_flags_t flags) { SET_FIELD(flags, flags); }
int injectorManager_setAcQueue(injectorManager *mgr, const char *name, inject_ac_t ac_queue) { SET_FIELD(ac_queue, ac_queue); }

int injectorManager_setPacketData(injectorManager *mgr, const char *name,
                                  const uint8_t *newData, uint32_t newLen) {
    if (!mgr || !name || !newData || newLen == 0) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = findInjectorByName(mgr, name);
    if (!inj) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_NOT_FOUND;
    }
    uint8_t *copy = pvPortMalloc(newLen);
    if (!copy) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_NO_SPACE;
    }
    memcpy(copy, newData, newLen);
    if (inj->packetData) vPortFree(inj->packetData);
    inj->packetData = copy;
    inj->packetLen = newLen;
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

int injectorManager_getInfo(injectorManager *mgr, const char *name, InjectorInfo *info) {
    if (!mgr || !name || !info) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = findInjectorByName(mgr, name);
    if (!inj) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_NOT_FOUND;
    }
    strncpy(info->name, inj->name, INJECTOR_NAME_MAX);
    info->name[INJECTOR_NAME_MAX - 1] = '\0';
    info->active = inj->active;
    info->channel = inj->channel;
    info->interval_ns = inj->interval_ns;
    info->maxPackets = inj->maxPackets;
    info->packets_sent = inj->packets_sent;
    info->tx_errors = inj->tx_errors;
    info->tx_retries = inj->swRetries_attempted;
    info->maxRetries = inj->hwRetries;
    info->tx_rate = inj->tx_rate;
    info->tx_power_dbm = inj->tx_power_dbm;
    info->flags = inj->flags;
    info->ac_queue = inj->ac_queue;
    info->packetLen = inj->packetLen;
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
    uint64_t total;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    total = mgr->totalPacketsAllTime;
    xSemaphoreGive(mgr->lock);
    return total;
}

uint32_t injectorManager_getActiveCount(injectorManager *mgr) {
    if (!mgr) return 0;
    uint32_t count = 0;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    for (int i = 0; i < INJECTOR_MAX; i++) {
        if (mgr->injectors[i].in_use && mgr->injectors[i].active) count++;
    }
    xSemaphoreGive(mgr->lock);
    return count;
}

uint32_t injectorManager_getTotalErrors(injectorManager *mgr) {
    if (!mgr) return 0;
    uint32_t errors;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    errors = mgr->totalErrorsAllTime;
    xSemaphoreGive(mgr->lock);
    return errors;
}

int injectorManager_startSchedulerTask(injectorManager *mgr, UBaseType_t priority) {
    return injectorManager_startSchedulerTaskEx(mgr, priority, 2048);
}

int injectorManager_startSchedulerTaskEx(injectorManager *mgr, UBaseType_t priority, uint32_t stackSize) {
    if (!mgr) return INJ_ERR_INVALID_ARG;
    if (mgr->scheduler_task != NULL) return INJ_ERR_STATE;
    mgr->scheduler_running = true;
    BaseType_t ret = xTaskCreate(schedulerTask, "inj_sched", stackSize, mgr, priority,
                                 &mgr->scheduler_task);
    if (ret != pdPASS) {
        mgr->scheduler_running = false;
        return INJ_ERR;
    }
    return INJ_OK;
}

int injectorManager_stopSchedulerTask(injectorManager *mgr) {
    if (!mgr) return INJ_ERR_INVALID_ARG;
    if (!mgr->scheduler_task) return INJ_ERR_STATE;

    mgr->scheduler_running = false;
    xSemaphoreGive(mgr->wake_sem);

    TickType_t timeout = pdMS_TO_TICKS(100);
    TickType_t start = xTaskGetTickCount();
    while (mgr->scheduler_task != NULL && (xTaskGetTickCount() - start) < timeout) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (mgr->scheduler_task != NULL) {
        vTaskDelete(mgr->scheduler_task);
        mgr->scheduler_task = NULL;
        mgr->scheduler_running = false;
    }

    return INJ_OK;
}
