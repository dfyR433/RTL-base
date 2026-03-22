/**
 * @file inject.c
 * @brief Implementation of the Packet Injector Manager (microsecond precision)
 */

#include "inject.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "wifi_api.h"
#include "wifi_api_types.h"
#include "wifi_api_ext.h"

/*============================================================================
 *                      Internal Data Structures
 *============================================================================*/
typedef struct {
    char name[INJECTOR_NAME_MAX];
    bool in_use;
    bool active;
    uint32_t generation;
    uint8_t channel;
    uint32_t interval_us;               /* microseconds */
    uint64_t next_time_us;               /* absolute time in microseconds */
    uint32_t maxPackets;
    uint32_t packets_sent;
    uint8_t  hwRetries;
    uint8_t  swRetries;
    uint8_t  swRetryCount;
    uint8_t  swBackoff;
    uint32_t tx_errors;
    int      last_error;
    inject_rate_t tx_rate;
    int8_t   tx_power_dbm;
    inject_flags_t flags;
    inject_ac_t ac_queue;
    uint8_t *packetData;
    uint32_t packetLen;
} PacketInjector;

struct injectorManager {
    PacketInjector injectors[INJECTOR_MAX];
    uint8_t injectorCount;
    uint64_t totalPacketsAllTime;
    int current_channel;
    SemaphoreHandle_t lock;
    TaskHandle_t scheduler_task;
    injector_timer_cb_t timer_callback;
};

/*============================================================================
 *                      Forward Declarations
 *============================================================================*/
static PacketInjector* findInjectorByName(injectorManager *mgr, const char *name);
static PacketInjector* findFreeSlot(injectorManager *mgr);
static void deleteInjector(injectorManager *mgr, PacketInjector *inj);
static int validateChannel(uint8_t channel);
static int validateRate(inject_rate_t rate);
static uint8_t rateToRtwRate(inject_rate_t rate);
static int sendPacket(PacketInjector *inj);
static uint64_t calcBackoffUs(uint8_t stage);
static PacketInjector* findInjectorBySlotAndGen(injectorManager *mgr,
                                                uint32_t slot,
                                                uint32_t generation);
static void schedulerTask(void *param);

/*============================================================================
 *                      Weak Default Implementations
 *============================================================================*/

/**
 * @brief Default implementation of platform_get_time_us.
 *        Uses DWT cycle counter if available; otherwise returns 0.
 *        Applications should provide their own high‑resolution timer.
 */
__attribute__((weak)) uint64_t platform_get_time_us(void) {
    static uint32_t cyccnt_per_us = 0;
    static uint32_t last_cycles = 0;
    static uint64_t us_offset = 0;

    taskENTER_CRITICAL();
    if (cyccnt_per_us == 0) {
        extern uint32_t SystemCoreClock;
        cyccnt_per_us = SystemCoreClock / 1000000;   /* cycles per µs */
        if (cyccnt_per_us == 0) cyccnt_per_us = 1;
    }
    uint32_t cycles = *((volatile uint32_t*)0xE0001004); /* DWT_CYCCNT */
    uint64_t delta;
    if (cycles < last_cycles) {
        delta = (0x100000000ULL - last_cycles) + cycles;
    } else {
        delta = (uint64_t)(cycles - last_cycles);
    }
    us_offset += delta / cyccnt_per_us;
    last_cycles = cycles;
    uint64_t ret = us_offset;
    taskEXIT_CRITICAL();
    return ret;
}

__attribute__((weak)) int platform_set_timer_us(uint64_t abs_time_us) {
    (void)abs_time_us;
    return -1;   /* not implemented */
}

/*============================================================================
 *                      Internal Helpers
 *============================================================================*/
static PacketInjector* findInjectorByName(injectorManager *mgr, const char *name) {
    for (int i = 0; i < INJECTOR_MAX; i++) {
        PacketInjector *inj = &mgr->injectors[i];
        if (inj->in_use && strcmp(inj->name, name) == 0) {
            return inj;
        }
    }
    return NULL;
}

static PacketInjector* findFreeSlot(injectorManager *mgr) {
    for (int i = 0; i < INJECTOR_MAX; i++) {
        if (!mgr->injectors[i].in_use) {
            return &mgr->injectors[i];
        }
    }
    return NULL;
}

static void deleteInjector(injectorManager *mgr, PacketInjector *inj) {
    if (!inj->in_use) return;
    inj->active = false;
#if INJECT_COPY_PACKET_DATA
    if (inj->packetData) {
        vPortFree(inj->packetData);
        inj->packetData = NULL;
    }
#endif
    inj->in_use = false;
    mgr->injectorCount--;
}

static int validateChannel(uint8_t channel) {
    if (channel >= 1 && channel <= 13) return INJ_OK;
    if (channel >= 36 && channel <= 165) return INJ_OK;
    return INJ_ERR_INVALID_ARG;
}

static int validateRate(inject_rate_t rate) {
    (void)rate;
    return INJ_OK;
}

static uint8_t rateToRtwRate(inject_rate_t rate) {
    return (uint8_t)rate;
}

static int sendPacket(PacketInjector *inj) {
    struct rtw_raw_frame_desc desc;
    memset(&desc, 0, sizeof(desc));
    desc.wlan_idx    = STA_WLAN_INDEX;
    desc.buf         = inj->packetData;
    desc.buf_len     = inj->packetLen;
    desc.tx_rate     = rateToRtwRate(inj->tx_rate);
    desc.retry_limit = inj->hwRetries;
    desc.ac_queue    = inj->ac_queue;
    desc.sgi         = (inj->flags & INJ_FLAG_USE_SHORT_GI) ? 1 : 0;
    desc.agg_en      = (inj->flags & INJ_FLAG_AGGREGATE) ? 1 : 0;
    return wifi_send_raw_frame(&desc);
}

static uint64_t calcBackoffUs(uint8_t stage) {
    if (stage > 10) stage = 10;
    uint32_t backoff_us = 1000U << stage;   /* 1ms, 2ms, 4ms... */
    if (backoff_us > 1000000U) backoff_us = 1000000U;
    return (uint64_t)backoff_us;
}

static PacketInjector* findInjectorBySlotAndGen(injectorManager *mgr,
                                                uint32_t slot,
                                                uint32_t generation) {
    if (!mgr || slot >= INJECTOR_MAX) return NULL;
    PacketInjector *inj = &mgr->injectors[slot];
    if (!inj->in_use || inj->generation != generation) return NULL;
    return inj;
}

/*============================================================================
 *                      Public API Implementation
 *============================================================================*/
injectorManager* injectorManager_create(void) {
    injectorManager *mgr = pvPortMalloc(sizeof(injectorManager));
    if (!mgr) return NULL;
    memset(mgr, 0, sizeof(injectorManager));
    mgr->lock = xSemaphoreCreateMutex();
    mgr->current_channel = -1;
    mgr->timer_callback = NULL;
    return mgr;
}

void injectorManager_destroy(injectorManager *mgr) {
    if (!mgr) return;
    injectorManager_clearAll(mgr);
    if (mgr->lock) vSemaphoreDelete(mgr->lock);
    vPortFree(mgr);
}

void injectorManager_clearAll(injectorManager *mgr) {
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    for (int i = 0; i < INJECTOR_MAX; i++) {
        PacketInjector *inj = &mgr->injectors[i];
        if (inj->in_use) {
            deleteInjector(mgr, inj);
        }
    }
    mgr->injectorCount = 0;
    mgr->current_channel = -1;
    xSemaphoreGive(mgr->lock);
}

int injectorManager_setInjectorEx(injectorManager *mgr,
                                  const char *name,
                                  const uint8_t *packetData,
                                  uint32_t packetLen,
                                  uint8_t channel,
                                  uint64_t start_time_us,
                                  uint32_t interval_us,
                                  uint32_t maxPackets,
                                  uint8_t hwRetries,
                                  uint8_t swRetries,
                                  inject_rate_t rate,
                                  int8_t tx_power_dbm,
                                  inject_flags_t flags,
                                  inject_ac_t ac_queue) {
    int ret;

    if (!mgr || !name || !packetData || packetLen == 0) return INJ_ERR_INVALID_ARG;
    if (strlen(name) >= INJECTOR_NAME_MAX) return INJ_ERR_INVALID_ARG;
    ret = validateChannel(channel);
    if (ret != INJ_OK) return ret;
    ret = validateRate(rate);
    if (ret != INJ_OK) return ret;

    xSemaphoreTake(mgr->lock, portMAX_DELAY);

    PacketInjector *existing = findInjectorByName(mgr, name);
    if (existing) {
        deleteInjector(mgr, existing);
    }

    PacketInjector *inj = findFreeSlot(mgr);
    if (!inj) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_NO_SPACE;
    }

#if INJECT_COPY_PACKET_DATA
    uint8_t *copy = pvPortMalloc(packetLen);
    if (!copy) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_NO_SPACE;
    }
    memcpy(copy, packetData, packetLen);
#else
    uint8_t *copy = (uint8_t*)packetData;
#endif

    uint32_t old_gen = inj->generation;
    memset(inj, 0, sizeof(PacketInjector));
    inj->generation = old_gen + 1;
    if (inj->generation == 0) inj->generation = 1;

    strncpy(inj->name, name, INJECTOR_NAME_MAX - 1);
    inj->name[INJECTOR_NAME_MAX - 1] = '\0';
    inj->in_use = true;
    inj->active = true;
    inj->channel = channel;
    inj->interval_us = interval_us;
    inj->maxPackets = maxPackets;
    inj->hwRetries = hwRetries;
    inj->swRetries = swRetries;
    inj->tx_rate = rate;
    inj->tx_power_dbm = tx_power_dbm;
    inj->flags = flags;
    inj->ac_queue = ac_queue;
    inj->packetData = copy;
    inj->packetLen = packetLen;

    uint64_t now = platform_get_time_us();
    inj->next_time_us = (start_time_us == 0 || start_time_us <= now) ? now : start_time_us;

    mgr->injectorCount++;

    if (mgr->scheduler_task != NULL) {
        vTaskResume(mgr->scheduler_task);
    }

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
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = findInjectorByName(mgr, name);
    if (!inj) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_NOT_FOUND;
    }
    inj->active = true;
    inj->next_time_us = platform_get_time_us();
    inj->swRetryCount = 0;
    inj->swBackoff = 0;
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

int injectorManager_deactivateInjector(injectorManager *mgr, const char *name) {
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

/*============================================================================
 *                      Dynamic Parameter Updates
 *============================================================================*/
#define SET_FIELD(field, value) \
    do { \
        xSemaphoreTake(mgr->lock, portMAX_DELAY); \
        PacketInjector *inj = findInjectorByName(mgr, name); \
        if (!inj) { \
            xSemaphoreGive(mgr->lock); \
            return INJ_ERR_NOT_FOUND; \
        } \
        inj->field = value; \
        xSemaphoreGive(mgr->lock); \
        return INJ_OK; \
    } while(0)

int injectorManager_setRate(injectorManager *mgr, const char *name, inject_rate_t rate) {
    int ret = validateRate(rate);
    if (ret != INJ_OK) return ret;
    SET_FIELD(tx_rate, rate);
}
int injectorManager_setChannel(injectorManager *mgr, const char *name, uint8_t channel) {
    int ret = validateChannel(channel);
    if (ret != INJ_OK) return ret;
    SET_FIELD(channel, channel);
}
int injectorManager_setTxPower(injectorManager *mgr, const char *name, int8_t tx_power_dbm) {
    SET_FIELD(tx_power_dbm, tx_power_dbm);
}
int injectorManager_setIntervalUs(injectorManager *mgr, const char *name, uint32_t interval_us) {
    SET_FIELD(interval_us, interval_us);
}
int injectorManager_setMaxPackets(injectorManager *mgr, const char *name, uint32_t maxPackets) {
    SET_FIELD(maxPackets, maxPackets);
}
int injectorManager_setHwRetries(injectorManager *mgr, const char *name, uint8_t hwRetries) {
    SET_FIELD(hwRetries, hwRetries);
}
int injectorManager_setSwRetries(injectorManager *mgr, const char *name, uint8_t swRetries) {
    SET_FIELD(swRetries, swRetries);
}
int injectorManager_setFlags(injectorManager *mgr, const char *name, inject_flags_t flags) {
    SET_FIELD(flags, flags);
}
int injectorManager_setAcQueue(injectorManager *mgr, const char *name, inject_ac_t ac_queue) {
    SET_FIELD(ac_queue, ac_queue);
}

int injectorManager_setPacketData(injectorManager *mgr, const char *name,
                                  const uint8_t *newData, uint32_t newLen) {
    if (!newData || newLen == 0) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = findInjectorByName(mgr, name);
    if (!inj) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_NOT_FOUND;
    }
#if INJECT_COPY_PACKET_DATA
    uint8_t *copy = pvPortMalloc(newLen);
    if (!copy) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_NO_SPACE;
    }
    memcpy(copy, newData, newLen);
    if (inj->packetData) vPortFree(inj->packetData);
    inj->packetData = copy;
#else
    inj->packetData = (uint8_t*)newData;
#endif
    inj->packetLen = newLen;
    xSemaphoreGive(mgr->lock);
    return INJ_OK;
}

/*============================================================================
 *                      Information and Status
 *============================================================================*/
int injectorManager_getInfo(injectorManager *mgr, const char *name, InjectorInfo *info) {
    if (!info) return INJ_ERR_INVALID_ARG;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    PacketInjector *inj = findInjectorByName(mgr, name);
    if (!inj) {
        xSemaphoreGive(mgr->lock);
        return INJ_ERR_NOT_FOUND;
    }
    strncpy(info->name, inj->name, INJECTOR_NAME_MAX);
    info->active = inj->active;
    info->channel = inj->channel;
    info->interval_us = inj->interval_us;
    info->maxPackets = inj->maxPackets;
    info->packets_sent = inj->packets_sent;
    info->maxRetries = inj->hwRetries;
    info->tx_errors = inj->tx_errors;
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
    if (!names || maxCount <= 0) return INJ_ERR_INVALID_ARG;
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
    uint64_t total;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    total = mgr->totalPacketsAllTime;
    xSemaphoreGive(mgr->lock);
    return total;
}

uint32_t injectorManager_getActiveCount(injectorManager *mgr) {
    uint32_t count = 0;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    for (int i = 0; i < INJECTOR_MAX; i++) {
        if (mgr->injectors[i].in_use && mgr->injectors[i].active) count++;
    }
    xSemaphoreGive(mgr->lock);
    return count;
}

/*============================================================================
 *                      Scheduling (Three‑Phase, microsecond)
 *============================================================================*/
int injectorManager_schedule(injectorManager *mgr, uint64_t now_us) {
    if (!mgr) return INJ_ERR_INVALID_ARG;

    typedef struct {
        uint32_t slot;
        uint32_t generation;
        uint8_t channel;
        uint8_t *packetData;
        uint32_t packetLen;
        inject_rate_t tx_rate;
        uint8_t hwRetries;
        inject_ac_t ac_queue;
        inject_flags_t flags;
        int8_t tx_power_dbm;
    } TxJob;

    TxJob jobs[INJECTOR_MAX];
    int job_count = 0;
    int packets_sent = 0;

    /* Phase 1: collect due jobs under lock */
    xSemaphoreTake(mgr->lock, portMAX_DELAY);

    for (int i = 0; i < INJECTOR_MAX; i++) {
        PacketInjector *inj = &mgr->injectors[i];
        if (!inj->in_use || !inj->active) continue;

        if (inj->maxPackets > 0 && inj->packets_sent >= inj->maxPackets) {
            deleteInjector(mgr, inj);
            continue;
        }

        if (inj->next_time_us > now_us) continue;

        if (job_count < INJECTOR_MAX) {
            jobs[job_count].slot         = (uint32_t)i;
            jobs[job_count].generation   = inj->generation;
            jobs[job_count].channel      = inj->channel;
            jobs[job_count].packetData   = inj->packetData;
            jobs[job_count].packetLen    = inj->packetLen;
            jobs[job_count].tx_rate      = inj->tx_rate;
            jobs[job_count].hwRetries    = inj->hwRetries;
            jobs[job_count].ac_queue     = inj->ac_queue;
            jobs[job_count].flags        = inj->flags;
            jobs[job_count].tx_power_dbm = inj->tx_power_dbm;
            job_count++;
        }
    }

    xSemaphoreGive(mgr->lock);

    /* Phase 2: transmit without the lock */
    int hw_channel = -1;
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    hw_channel = mgr->current_channel;
    xSemaphoreGive(mgr->lock);

    for (int j = 0; j < job_count; j++) {
        const TxJob *job = &jobs[j];
        int ret = 0;

        if (!(job->flags & INJ_FLAG_FIXED_CHANNEL) && hw_channel != (int)job->channel) {
            ret = wifi_set_channel(STA_WLAN_INDEX, job->channel);
            if (ret == 0) {
                hw_channel = job->channel;
                xSemaphoreTake(mgr->lock, portMAX_DELAY);
                mgr->current_channel = hw_channel;
                xSemaphoreGive(mgr->lock);
            } else {
                ret = INJ_ERR_CHANNEL;
            }
        }

        if (ret == 0) {
            PacketInjector temp_inj = {
                .packetData   = job->packetData,
                .packetLen    = job->packetLen,
                .tx_rate      = job->tx_rate,
                .hwRetries    = job->hwRetries,
                .ac_queue     = job->ac_queue,
                .flags        = job->flags,
                .tx_power_dbm = job->tx_power_dbm
            };
            ret = sendPacket(&temp_inj);
        }

        /* Phase 3: re‑lock and update the injector */
        xSemaphoreTake(mgr->lock, portMAX_DELAY);
        PacketInjector *inj = findInjectorBySlotAndGen(mgr, job->slot, job->generation);
        if (!inj) {
            xSemaphoreGive(mgr->lock);
            continue;
        }

        if (ret == 0) {
            inj->packets_sent++;
            mgr->totalPacketsAllTime++;
            packets_sent++;
            inj->swRetryCount = 0;
            inj->swBackoff    = 0;
            inj->last_error   = 0;

            uint64_t interval = inj->interval_us ? (uint64_t)inj->interval_us : 1ULL;
            uint64_t next = inj->next_time_us;
            if (next <= now_us) next = now_us;
            next += interval;
            while (next <= now_us) next += interval;
            inj->next_time_us = next;

        } else if (ret == INJ_ERR_CHANNEL || ret == -RTK_ERR_WIFI_TX_BUF_FULL) {
            inj->tx_errors++;
            inj->last_error = (ret == INJ_ERR_CHANNEL) ? INJ_ERR_CHANNEL : INJ_ERR_BUSY;
            inj->swRetryCount++;

            uint64_t backoff_us = calcBackoffUs(inj->swBackoff);
            inj->next_time_us = now_us + backoff_us;
            if (inj->swBackoff < 5) inj->swBackoff++;

            if (inj->swRetries > 0 && inj->swRetryCount > inj->swRetries) {
                deleteInjector(mgr, inj);
            }
        } else {
            inj->tx_errors++;
            inj->last_error = ret;
            deleteInjector(mgr, inj);
        }

        xSemaphoreGive(mgr->lock);
    }

    /* Phase 4: compute next wake time */
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    uint64_t next_wake = UINT64_MAX;
    for (int i = 0; i < INJECTOR_MAX; i++) {
        PacketInjector *inj = &mgr->injectors[i];
        if (!inj->in_use || !inj->active) continue;
        if (inj->next_time_us < next_wake) next_wake = inj->next_time_us;
    }

    if (next_wake == UINT64_MAX) {
        if (mgr->scheduler_task != NULL) {
            vTaskSuspend(mgr->scheduler_task);
        }
        xSemaphoreGive(mgr->lock);
        return 0;
    }
    xSemaphoreGive(mgr->lock);

    if (platform_set_timer_us(next_wake) != 0) {
        return INJ_ERR_TIMER;
    }

    return packets_sent;
}

/*============================================================================
 *                      RTOS Task and Timer ISR
 *============================================================================*/
static void schedulerTask(void *param) {
    injectorManager *mgr = (injectorManager*)param;
    uint64_t now;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        now = platform_get_time_us();
        injectorManager_schedule(mgr, now);
    }
}

void injectorManager_timerIsr(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    extern injectorManager *g_inj_mgr;
    if (g_inj_mgr && g_inj_mgr->timer_callback) {
        g_inj_mgr->timer_callback(g_inj_mgr);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void injectorManager_setTimerCallback(injectorManager *mgr, injector_timer_cb_t callback) {
    xSemaphoreTake(mgr->lock, portMAX_DELAY);
    mgr->timer_callback = callback;
    xSemaphoreGive(mgr->lock);
}

static void defaultTimerCallback(injectorManager *mgr) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (mgr->scheduler_task != NULL) {
        vTaskNotifyGiveFromISR(mgr->scheduler_task, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int injectorManager_startSchedulerTask(injectorManager *mgr, UBaseType_t priority) {
    if (!mgr) return INJ_ERR_INVALID_ARG;
    BaseType_t ret = xTaskCreate(schedulerTask, "inj_sched", 2048, mgr, priority,
                                 &mgr->scheduler_task);
    if (ret != pdPASS) return INJ_ERR;
    injectorManager_setTimerCallback(mgr, defaultTimerCallback);
    return INJ_OK;
}
