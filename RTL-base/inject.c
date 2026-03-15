#include "inject.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "wifi_api.h"
#include "wifi_api_types.h"
#include "wifi_api_ext.h"

__attribute__((weak)) uint64_t platform_get_time_ns(void) {
    extern uint32_t SystemCoreClock;
    static uint32_t cyccnt_per_ns = 0;
    if (cyccnt_per_ns == 0) {
        cyccnt_per_ns = SystemCoreClock / 1000000;
        if (cyccnt_per_ns == 0) cyccnt_per_ns = 1;
    }
    uint32_t cycles = *((volatile uint32_t*)0xE0001004);
    static uint64_t last_cycles = 0, ns_offset = 0;
    if (cycles < last_cycles) {
        ns_offset += (0xFFFFFFFFULL - last_cycles + cycles) * 1000 / cyccnt_per_ns;
    } else {
        ns_offset += (cycles - last_cycles) * 1000 / cyccnt_per_ns;
    }
    last_cycles = cycles;
    return ns_offset;
}

__attribute__((weak)) int platform_set_timer_ns(uint64_t abs_time_ns) {
    (void)abs_time_ns;
    return -1;
}

static PacketInjector* find_injector_by_name(injectorManager *mgr, const char *name) {
    for (int i = 0; i < INJECTOR_MAX; i++) {
        if (mgr->injectors[i].active && strcmp(mgr->injectors[i].name, name) == 0) {
            return &mgr->injectors[i];
        }
    }
    return NULL;
}

static PacketInjector* find_free_slot(injectorManager *mgr) {
    for (int i = 0; i < INJECTOR_MAX; i++) {
        if (!mgr->injectors[i].active) {
            return &mgr->injectors[i];
        }
    }
    return NULL;
}

#ifndef RTW_RATE_1M
#define RTW_RATE_1M     0x02
#define RTW_RATE_2M     0x04
#define RTW_RATE_5_5M   0x0B
#define RTW_RATE_11M    0x16
#define RTW_RATE_6M     0x0C
#define RTW_RATE_9M     0x12
#define RTW_RATE_12M    0x18
#define RTW_RATE_18M    0x24
#define RTW_RATE_24M    0x30
#define RTW_RATE_36M    0x48
#define RTW_RATE_48M    0x60
#define RTW_RATE_54M    0x6C
#define RTW_RATE_MCS0   0x80
#define RTW_RATE_MCS1   0x81
#define RTW_RATE_MCS2   0x82
#define RTW_RATE_MCS3   0x83
#define RTW_RATE_MCS4   0x84
#define RTW_RATE_MCS5   0x85
#define RTW_RATE_MCS6   0x86
#define RTW_RATE_MCS7   0x87
#endif

static uint8_t rate_to_rtw_rate(inject_rate_t rate) {
    switch (rate) {
        case INJ_RATE_1M:   return RTW_RATE_1M;
        case INJ_RATE_2M:   return RTW_RATE_2M;
        case INJ_RATE_5_5M: return RTW_RATE_5_5M;
        case INJ_RATE_11M:  return RTW_RATE_11M;
        case INJ_RATE_6M:   return RTW_RATE_6M;
        case INJ_RATE_9M:   return RTW_RATE_9M;
        case INJ_RATE_12M:  return RTW_RATE_12M;
        case INJ_RATE_18M:  return RTW_RATE_18M;
        case INJ_RATE_24M:  return RTW_RATE_24M;
        case INJ_RATE_36M:  return RTW_RATE_36M;
        case INJ_RATE_48M:  return RTW_RATE_48M;
        case INJ_RATE_54M:  return RTW_RATE_54M;
        case INJ_RATE_MCS0: return RTW_RATE_MCS0;
        case INJ_RATE_MCS1: return RTW_RATE_MCS1;
        case INJ_RATE_MCS2: return RTW_RATE_MCS2;
        case INJ_RATE_MCS3: return RTW_RATE_MCS3;
        case INJ_RATE_MCS4: return RTW_RATE_MCS4;
        case INJ_RATE_MCS5: return RTW_RATE_MCS5;
        case INJ_RATE_MCS6: return RTW_RATE_MCS6;
        case INJ_RATE_MCS7: return RTW_RATE_MCS7;
        default:            return RTW_RATE_1M;
    }
}

static int send_packet(PacketInjector *inj) {
    struct rtw_raw_frame_desc desc;
    memset(&desc, 0, sizeof(desc));
    desc.wlan_idx = STA_WLAN_INDEX;
    desc.buf = inj->packetData;
    desc.buf_len = inj->packetLen;
    desc.tx_rate = rate_to_rtw_rate(inj->tx_rate);
    desc.retry_limit = inj->maxRetries;
    desc.ac_queue = 0;
    desc.sgi = (inj->flags & INJ_FLAG_USE_SHORT_GI) ? 1 : 0;
    desc.agg_en = (inj->flags & INJ_FLAG_AGGREGATE) ? 1 : 0;
    return wifi_send_raw_frame(&desc);
}

void injectorManager_init(injectorManager *mgr) {
    memset(mgr, 0, sizeof(injectorManager));
    MGR_LOCK_INIT(mgr);
    mgr->injectorCount = 0;
    mgr->current_channel = -1;
}

void injectorManager_clearAllInjectors(injectorManager *mgr) {
    MGR_LOCK(mgr);
    for (int i = 0; i < INJECTOR_MAX; i++) {
        PacketInjector *inj = &mgr->injectors[i];
        if (inj->active) {
#if INJECT_COPY_PACKET_DATA
            if (inj->packetData) {
                vPortFree(inj->packetData);
                inj->packetData = NULL;
            }
#endif
            inj->active = false;
        }
    }
    mgr->injectorCount = 0;
    MGR_UNLOCK(mgr);
}

int injectorManager_startInjectorEx(injectorManager *mgr,
                                    const char *injectorName,
                                    const uint8_t *packetData,
                                    uint32_t packetLen,
                                    uint8_t channel,
                                    uint64_t start_time_ns,
                                    uint32_t interval_ns,
                                    uint32_t maxPackets,
                                    uint8_t maxRetries,
                                    inject_rate_t rate,
                                    int8_t tx_power_dbm,
                                    inject_flags_t flags) {
    if (!mgr || !injectorName || !packetData || packetLen == 0) {
        return INJ_ERR_INVALID_ARG;
    }
    if (strlen(injectorName) >= INJECTOR_NAME_MAX) {
        return INJ_ERR_INVALID_ARG;
    }

    MGR_LOCK(mgr);

    if (find_injector_by_name(mgr, injectorName) != NULL) {
        MGR_UNLOCK(mgr);
        return INJ_ERR;
    }

    PacketInjector *inj = find_free_slot(mgr);
    if (!inj) {
        MGR_UNLOCK(mgr);
        return INJ_ERR_NO_SPACE;
    }

    memset(inj, 0, sizeof(PacketInjector));
    strncpy(inj->name, injectorName, INJECTOR_NAME_MAX - 1);
    inj->name[INJECTOR_NAME_MAX - 1] = '\0';

#if INJECT_COPY_PACKET_DATA
    uint8_t *copy = pvPortMalloc(packetLen);
    if (!copy) {
        MGR_UNLOCK(mgr);
        return INJ_ERR_NO_SPACE;
    }
    memcpy(copy, packetData, packetLen);
    inj->packetData = copy;
#else
    inj->packetData = (uint8_t*)packetData;
#endif
    inj->packetLen = packetLen;
    inj->channel = channel;
    inj->interval_ns = interval_ns;
    inj->maxPackets = maxPackets;
    inj->maxRetries = maxRetries;
    inj->tx_rate = rate;
    inj->tx_power_dbm = tx_power_dbm;
    inj->flags = flags;
    inj->packets_sent = 0;
    inj->retry_count = 0;
    inj->active = true;

    uint64_t now = platform_get_time_ns();
    if (start_time_ns == 0 || start_time_ns <= now) {
        inj->next_time_ns = now;
    } else {
        inj->next_time_ns = start_time_ns;
    }

    mgr->injectorCount++;

#ifdef INJECT_USE_RTOS
    if (mgr->scheduler_task != NULL) {
        vTaskResume(mgr->scheduler_task);
    }
#endif

    MGR_UNLOCK(mgr);
    return INJ_OK;
}

int injectorManager_stopInjector(injectorManager *mgr, const char *injectorName) {
    if (!mgr || !injectorName) return INJ_ERR_INVALID_ARG;

    MGR_LOCK(mgr);
    PacketInjector *inj = find_injector_by_name(mgr, injectorName);
    if (!inj) {
        MGR_UNLOCK(mgr);
        return INJ_ERR_NOT_FOUND;
    }

    inj->active = false;
#if INJECT_COPY_PACKET_DATA
    if (inj->packetData) {
        vPortFree(inj->packetData);
        inj->packetData = NULL;
    }
#endif
    mgr->injectorCount--;
    MGR_UNLOCK(mgr);
    return INJ_OK;
}

int injectorManager_activateInjector(injectorManager *mgr, const char *injectorName) {
    MGR_LOCK(mgr);
    PacketInjector *inj = find_injector_by_name(mgr, injectorName);
    if (!inj) {
        MGR_UNLOCK(mgr);
        return INJ_ERR_NOT_FOUND;
    }
    inj->active = true;
    inj->next_time_ns = platform_get_time_ns();
    MGR_UNLOCK(mgr);
    return INJ_OK;
}

int injectorManager_deactivateInjector(injectorManager *mgr, const char *injectorName) {
    MGR_LOCK(mgr);
    PacketInjector *inj = find_injector_by_name(mgr, injectorName);
    if (!inj) {
        MGR_UNLOCK(mgr);
        return INJ_ERR_NOT_FOUND;
    }
    inj->active = false;
    MGR_UNLOCK(mgr);
    return INJ_OK;
}

#define SET_INJECTOR_FIELD(mgr, name, field, value) \
    do { \
        MGR_LOCK(mgr); \
        PacketInjector *inj = find_injector_by_name(mgr, name); \
        if (!inj) { \
            MGR_UNLOCK(mgr); \
            return INJ_ERR_NOT_FOUND; \
        } \
        inj->field = value; \
        MGR_UNLOCK(mgr); \
        return INJ_OK; \
    } while(0)

int injectorManager_setInjectorRate(injectorManager *mgr, const char *injectorName, inject_rate_t rate) {
    SET_INJECTOR_FIELD(mgr, injectorName, tx_rate, rate);
}
int injectorManager_setInjectorChannel(injectorManager *mgr, const char *injectorName, uint8_t channel) {
    SET_INJECTOR_FIELD(mgr, injectorName, channel, channel);
}
int injectorManager_setInjectorPower(injectorManager *mgr, const char *injectorName, int8_t tx_power_dbm) {
    SET_INJECTOR_FIELD(mgr, injectorName, tx_power_dbm, tx_power_dbm);
}
int injectorManager_setInjectorIntervalNs(injectorManager *mgr, const char *injectorName, uint32_t interval_ns) {
    SET_INJECTOR_FIELD(mgr, injectorName, interval_ns, interval_ns);
}
int injectorManager_setInjectorMaxPackets(injectorManager *mgr, const char *injectorName, uint32_t maxPackets) {
    SET_INJECTOR_FIELD(mgr, injectorName, maxPackets, maxPackets);
}
int injectorManager_setInjectorRetries(injectorManager *mgr, const char *injectorName, uint8_t maxRetries) {
    SET_INJECTOR_FIELD(mgr, injectorName, maxRetries, maxRetries);
}
int injectorManager_setInjectorFlags(injectorManager *mgr, const char *injectorName, inject_flags_t flags) {
    SET_INJECTOR_FIELD(mgr, injectorName, flags, flags);
}

int injectorManager_setInjectorPacketData(injectorManager *mgr, const char *injectorName,
                                          const uint8_t *newData, uint32_t newLen) {
    if (!newData || newLen == 0) return INJ_ERR_INVALID_ARG;

    MGR_LOCK(mgr);
    PacketInjector *inj = find_injector_by_name(mgr, injectorName);
    if (!inj) {
        MGR_UNLOCK(mgr);
        return INJ_ERR_NOT_FOUND;
    }

#if INJECT_COPY_PACKET_DATA
    uint8_t *copy = pvPortMalloc(newLen);
    if (!copy) {
        MGR_UNLOCK(mgr);
        return INJ_ERR_NO_SPACE;
    }
    memcpy(copy, newData, newLen);
    if (inj->packetData) {
        vPortFree(inj->packetData);
    }
    inj->packetData = copy;
#else
    inj->packetData = (uint8_t*)newData;
#endif
    inj->packetLen = newLen;
    MGR_UNLOCK(mgr);
    return INJ_OK;
}

int injectorManager_getInjectorInfo(injectorManager *mgr, const char *injectorName, InjectorInfo *info) {
    if (!info) return INJ_ERR_INVALID_ARG;

    MGR_LOCK(mgr);
    PacketInjector *inj = find_injector_by_name(mgr, injectorName);
    if (!inj) {
        MGR_UNLOCK(mgr);
        return INJ_ERR_NOT_FOUND;
    }

    strncpy(info->name, inj->name, INJECTOR_NAME_MAX);
    info->active = inj->active;
    info->channel = inj->channel;
    info->interval_ns = inj->interval_ns;
    info->maxPackets = inj->maxPackets;
    info->packets_sent = inj->packets_sent;
    info->maxRetries = inj->maxRetries;
    info->tx_rate = inj->tx_rate;
    info->tx_power_dbm = inj->tx_power_dbm;
    info->flags = inj->flags;
    info->packetLen = inj->packetLen;

    MGR_UNLOCK(mgr);
    return INJ_OK;
}

int injectorManager_listInjectors(injectorManager *mgr, char names[][INJECTOR_NAME_MAX], int maxCount) {
    if (!names || maxCount <= 0) return INJ_ERR_INVALID_ARG;

    MGR_LOCK(mgr);
    int count = 0;
    for (int i = 0; i < INJECTOR_MAX && count < maxCount; i++) {
        if (mgr->injectors[i].active) {
            strncpy(names[count], mgr->injectors[i].name, INJECTOR_NAME_MAX);
            names[count][INJECTOR_NAME_MAX - 1] = '\0';
            count++;
        }
    }
    MGR_UNLOCK(mgr);
    return count;
}

int injectorManager_schedule(injectorManager *mgr, uint64_t now_ns) {
    if (!mgr) return INJ_ERR_INVALID_ARG;

    MGR_LOCK(mgr);

    uint64_t next_wake = UINT64_MAX;
    int any_sent = 0;

    for (int i = 0; i < INJECTOR_MAX; i++) {
        PacketInjector *inj = &mgr->injectors[i];
        if (!inj->active) continue;

        if (inj->maxPackets > 0 && inj->packets_sent >= inj->maxPackets) {
            inj->active = false;
#if INJECT_COPY_PACKET_DATA
            if (inj->packetData) {
                vPortFree(inj->packetData);
                inj->packetData = NULL;
            }
#endif
            mgr->injectorCount--;
            continue;
        }

        if (inj->next_time_ns > now_ns) {
            if (inj->next_time_ns < next_wake) next_wake = inj->next_time_ns;
            continue;
        }

        if (!(inj->flags & INJ_FLAG_FIXED_CHANNEL) && mgr->current_channel != inj->channel) {
            if (wifi_set_channel(STA_WLAN_INDEX, inj->channel) == 0) {
                mgr->current_channel = inj->channel;
            } else {
                if (inj->next_time_ns < next_wake) next_wake = inj->next_time_ns;
                continue;
            }
        }

        int ret = send_packet(inj);
        if (ret == 0) {
            inj->packets_sent++;
            mgr->totalPacketsThisRun++;
            mgr->totalPacketsAllTime++;
            inj->retry_count = 0;
            any_sent = 1;

            inj->next_time_ns += inj->interval_ns;
            while (inj->next_time_ns <= now_ns) {
                inj->next_time_ns += inj->interval_ns;
            }
            if (inj->next_time_ns < next_wake) next_wake = inj->next_time_ns;
        } else if (ret == -RTK_ERR_WIFI_TX_BUF_FULL) {
            inj->retry_count++;
            if (inj->maxRetries > 0 && inj->retry_count > inj->maxRetries) {
                inj->active = false;
#if INJECT_COPY_PACKET_DATA
                if (inj->packetData) {
                    vPortFree(inj->packetData);
                    inj->packetData = NULL;
                }
#endif
                mgr->injectorCount--;
            } else {
                if (inj->next_time_ns < next_wake) next_wake = inj->next_time_ns;
            }
        } else {
            inj->active = false;
#if INJECT_COPY_PACKET_DATA
            if (inj->packetData) {
                vPortFree(inj->packetData);
                inj->packetData = NULL;
            }
#endif
            mgr->injectorCount--;
        }
    }

    MGR_UNLOCK(mgr);

    if (next_wake == UINT64_MAX) {
#ifdef INJECT_USE_RTOS
        if (mgr->scheduler_task != NULL) {
            vTaskSuspend(mgr->scheduler_task);
        }
#endif
        return INJ_ERR_NOT_FOUND;
    }

    int ret = platform_set_timer_ns(next_wake);
    if (ret != 0) {
        return INJ_ERR_TIMER;
    }

    return any_sent ? INJ_OK : INJ_ERR;
}

#ifdef INJECT_USE_RTOS

static void scheduler_task(void *param) {
    injectorManager *mgr = (injectorManager*)param;
    uint64_t now;
    int res;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        now = platform_get_time_ns();
        res = injectorManager_schedule(mgr, now);
        if (res == INJ_ERR_NOT_FOUND) {
            vTaskSuspend(NULL);
        }
    }
}

void injector_timer_isr(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    extern injectorManager g_inj_mgr;
    if (g_inj_mgr.scheduler_task != NULL) {
        vTaskNotifyGiveFromISR(g_inj_mgr.scheduler_task, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int injectorManager_startSchedulerTask(injectorManager *mgr, UBaseType_t priority) {
    BaseType_t ret = xTaskCreate(scheduler_task, "inj_sched", 2048, mgr, priority, &mgr->scheduler_task);
    if (ret != pdPASS) return INJ_ERR;
    return INJ_OK;
}

#endif
