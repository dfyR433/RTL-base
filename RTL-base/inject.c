/**
 * @file inject.c
 * @brief Packet injector manager.
 */

#include "inject.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "wifi_api.h"
#include "wifi_api_types.h"
#include "wifi_api_ext.h"

/*============================================================================
 * Internal data structures
 *============================================================================*/
typedef struct {
    char name[INJECTOR_NAME_MAX];
    bool in_use;
    bool active;
    uint8_t channel;
    uint64_t interval_ns;
    uint64_t next_time_ns;
    uint32_t maxPackets;
    uint32_t packets_sent;
    uint8_t  hwRetries;
    uint8_t  swRetries;
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
    int8_t current_tx_power;
    uint8_t current_channel;
    SemaphoreHandle_t lock;
    TaskHandle_t scheduler_task;
};

static injectorManager *g_inj_mgr = NULL;

/*============================================================================
 * High‑resolution timer – nanosecond precision via platform‑provided timer
 *============================================================================*/

uint32_t timer_freq_hz = 0;
static uint32_t last_timer_count = 0;
static uint64_t time_offset_ns = 0;

/* These functions must be implemented for your platform. */
extern void init_highres_timer(void);
extern uint32_t read_highres_timer(void);

uint64_t platform_get_time_ns(void) {
    taskENTER_CRITICAL();

    if (timer_freq_hz == 0) {
        timer_freq_hz = 1000000;
    }

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
 * Percentage‑based TX power mapping (stub)
 *============================================================================*/
static uint8_t dbm_to_percentage(int8_t dbm) {
    if (dbm <= 0)           return 4;   /* 12.5% */
    if (dbm <= 5)           return 3;   /* 25%   */
    if (dbm <= 10)          return 2;   /* 50%   */
    if (dbm <= 15)          return 1;   /* 75%   */
    return 0;                           /* 100%  */
}

__attribute__((weak)) int wifi_set_tx_power_percentage(u8 wlan_idx, uint8_t percentage) {
    (void)wlan_idx; (void)percentage;
    return 0;
}

static int set_tx_power_percentage(u8 wlan_idx, int8_t power_dbm) {
    if (power_dbm < 0) return 0;
    uint8_t perc = dbm_to_percentage(power_dbm);
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
    mgr->injectorCount--;
}

static int sendPacket(PacketInjector *inj) {
    struct rtw_raw_frame_desc desc = {0};
    desc.wlan_idx    = STA_WLAN_INDEX;
    desc.buf         = inj->packetData;
    desc.buf_len     = inj->packetLen;
    desc.tx_rate     = (u8)inj->tx_rate;
    desc.retry_limit = inj->hwRetries;
    desc.ac_queue    = inj->ac_queue;
    desc.sgi         = (inj->flags & INJ_FLAG_USE_SHORT_GI) ? 1 : 0;
    desc.agg_en      = (inj->flags & INJ_FLAG_AGGREGATE) ? 1 : 0;

    int ret = wifi_send_raw_frame(&desc);
    if (ret != 0) {
        printf("sendPacket error: %d\n", ret);
    }
    return ret;
}

/*============================================================================
 * Scheduler task (polling every 1 ms)
 *============================================================================*/
static void schedulerTask(void *param) {
    injectorManager *mgr = (injectorManager*)param;
    uint64_t now;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1));
        now = platform_get_time_ns();

        xSemaphoreTake(mgr->lock, portMAX_DELAY);

        for (int i = 0; i < INJECTOR_MAX; i++) {
            PacketInjector *inj = &mgr->injectors[i];
            if (!inj->in_use || !inj->active) continue;

            if (inj->maxPackets > 0 && inj->packets_sent >= inj->maxPackets) {
                deleteInjector(mgr, inj);
                continue;
            }

            if (inj->next_time_ns > now) continue;

            /* Set channel if needed (only if injector has a specific channel) */
            if (inj->channel != 0 && inj->channel != mgr->current_channel) {
                if (wifi_set_channel(STA_WLAN_INDEX, inj->channel) == 0) {
                    mgr->current_channel = inj->channel;
                } else {
                    /* Retry after 1 ms */
                    inj->next_time_ns = now + 1000000ULL;
                    continue;
                }
            }

            /* Set TX power globally (if changed) – we use the first injector's power */
            if (inj->tx_power_dbm != -1 && inj->tx_power_dbm != mgr->current_tx_power) {
                if (set_tx_power_percentage(STA_WLAN_INDEX, inj->tx_power_dbm) == 0) {
                    mgr->current_tx_power = inj->tx_power_dbm;
                } else {
                    inj->next_time_ns = now + 1000000ULL;
                    continue;
                }
            }

            int ret = sendPacket(inj);
            if (ret == 0) {
                inj->packets_sent++;
                mgr->totalPacketsAllTime++;
                inj->next_time_ns = now + inj->interval_ns;
            } else {
                /* Retry after a short delay */
                inj->next_time_ns = now + 1000000ULL;
            }
        }

        xSemaphoreGive(mgr->lock);
    }
}

/*============================================================================
 * Public API
 *============================================================================*/
injectorManager* injectorManager_create(void) {
    injectorManager *mgr = pvPortMalloc(sizeof(injectorManager));
    if (!mgr) return NULL;
    memset(mgr, 0, sizeof(injectorManager));
    mgr->lock = xSemaphoreCreateMutex();
    if (!mgr->lock) {
        vPortFree(mgr);
        return NULL;
    }
    mgr->current_tx_power = -1;
    mgr->current_channel = 0;

    init_highres_timer();
    last_timer_count = read_highres_timer();
    time_offset_ns = 0;

    g_inj_mgr = mgr;
    return mgr;
}

void injectorManager_destroy(injectorManager *mgr) {
    if (!mgr) return;
    injectorManager_clearAll(mgr);
    if (mgr->lock) vSemaphoreDelete(mgr->lock);
    vPortFree(mgr);
    if (g_inj_mgr == mgr) g_inj_mgr = NULL;
}

void injectorManager_clearAll(injectorManager *mgr) {
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
    inj->tx_power_dbm = tx_power_dbm;
    inj->tx_rate = rate;
    inj->flags = flags;
    inj->ac_queue = ac_queue;
    inj->packetData = copy;
    inj->packetLen = packetLen;

    uint64_t now = platform_get_time_ns();
    inj->next_time_ns = (start_time_ns == 0 || start_time_ns <= now) ? now : start_time_ns;

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
    if (!newData || newLen == 0) return INJ_ERR_INVALID_ARG;
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
    info->interval_ns = inj->interval_ns;
    info->maxPackets = inj->maxPackets;
    info->packets_sent = inj->packets_sent;
    info->maxRetries = inj->hwRetries;
    info->tx_errors = 0; // not tracked
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

int injectorManager_startSchedulerTask(injectorManager *mgr, UBaseType_t priority) {
    if (!mgr) return INJ_ERR_INVALID_ARG;
    if (mgr->scheduler_task != NULL) return INJ_ERR_STATE;
    BaseType_t ret = xTaskCreate(schedulerTask, "inj_sched", 2048, mgr, priority,
                                 &mgr->scheduler_task);
    return (ret == pdPASS) ? INJ_OK : INJ_ERR;
}
