#ifndef INJECT_H
#define INJECT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INJ_OK              0
#define INJ_ERR             -1
#define INJ_ERR_NOT_FOUND   -2
#define INJ_ERR_INVALID_ARG -3
#define INJ_ERR_NO_SPACE    -4
#define INJ_ERR_TIMER       -5
#define INJ_ERR_CHANNEL     -6
#define INJ_ERR_BUSY        -7

#ifdef INJECT_USE_RTOS
    #include <freertos/FreeRTOS.h>
    #include <freertos/semphr.h>
    #include <freertos/task.h>
    #define MGR_LOCK_INIT(mgr)   do { (mgr)->lock = xSemaphoreCreateMutex(); } while(0)
    #define MGR_LOCK_FREE(mgr)   vSemaphoreDelete((mgr)->lock)
    #define MGR_LOCK(mgr)        xSemaphoreTake((mgr)->lock, portMAX_DELAY)
    #define MGR_UNLOCK(mgr)      xSemaphoreGive((mgr)->lock)
#else
    #define MGR_LOCK_INIT(mgr)    ((void)0)
    #define MGR_LOCK_FREE(mgr)    ((void)0)
    #define MGR_LOCK(mgr)         ((void)0)
    #define MGR_UNLOCK(mgr)       ((void)0)
#endif

#define INJECTOR_NAME_MAX   32
#define INJECTOR_MAX        16

#ifndef INJECT_COPY_PACKET_DATA
#define INJECT_COPY_PACKET_DATA 1
#endif

typedef enum {
    INJ_RATE_1M,
    INJ_RATE_2M,
    INJ_RATE_5_5M,
    INJ_RATE_11M,
    INJ_RATE_6M,
    INJ_RATE_9M,
    INJ_RATE_12M,
    INJ_RATE_18M,
    INJ_RATE_24M,
    INJ_RATE_36M,
    INJ_RATE_48M,
    INJ_RATE_54M,
    INJ_RATE_MCS0,
    INJ_RATE_MCS1,
    INJ_RATE_MCS2,
    INJ_RATE_MCS3,
    INJ_RATE_MCS4,
    INJ_RATE_MCS5,
    INJ_RATE_MCS6,
    INJ_RATE_MCS7,
    INJ_RATE_DEFAULT = INJ_RATE_1M
} inject_rate_t;

typedef enum {
    INJ_FLAG_NONE           = 0,
    INJ_FLAG_NO_ACK         = (1 << 0),
    INJ_FLAG_USE_SHORT_GI   = (1 << 1),
    INJ_FLAG_AGGREGATE      = (1 << 2),
    INJ_FLAG_FIXED_CHANNEL  = (1 << 3),
} inject_flags_t;

typedef struct {
    char name[INJECTOR_NAME_MAX];
    bool active;
    uint8_t channel;
    uint32_t interval_ns;
    uint64_t next_time_ns;
    uint64_t start_time_ns;
    uint32_t maxPackets;
    uint32_t packets_sent;
    uint8_t  maxRetries;
    uint8_t  retry_count;
    inject_rate_t tx_rate;
    int8_t   tx_power_dbm;
    inject_flags_t flags;
    uint8_t *packetData;
    uint32_t packetLen;
} PacketInjector;

typedef struct {
    char name[INJECTOR_NAME_MAX];
    bool active;
    uint8_t channel;
    uint32_t interval_ns;
    uint32_t maxPackets;
    uint32_t packets_sent;
    uint8_t  maxRetries;
    inject_rate_t tx_rate;
    int8_t   tx_power_dbm;
    inject_flags_t flags;
    uint32_t packetLen;
} InjectorInfo;

typedef struct {
    PacketInjector injectors[INJECTOR_MAX];
    uint8_t injectorCount;
    uint64_t totalPacketsThisRun;
    uint64_t totalPacketsAllTime;
    int current_channel;
#ifdef INJECT_USE_RTOS
    SemaphoreHandle_t lock;
    TaskHandle_t scheduler_task;
#endif
} injectorManager;

void injectorManager_init(injectorManager *mgr);
void injectorManager_clearAllInjectors(injectorManager *mgr);

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
                                    inject_flags_t flags);

static inline int injectorManager_startInjector(injectorManager *mgr,
                                                const char *injectorName,
                                                const uint8_t *packetData,
                                                uint32_t packetLen,
                                                uint8_t channel,
                                                uint32_t interval_us,
                                                uint32_t maxPackets,
                                                uint8_t maxRetries) {
    return injectorManager_startInjectorEx(mgr, injectorName, packetData, packetLen,
                                           channel, 0, interval_us * 1000,
                                           maxPackets, maxRetries,
                                           INJ_RATE_DEFAULT, -1, INJ_FLAG_NONE);
}

int injectorManager_stopInjector(injectorManager *mgr, const char *injectorName);
int injectorManager_activateInjector(injectorManager *mgr, const char *injectorName);
int injectorManager_deactivateInjector(injectorManager *mgr, const char *injectorName);

int injectorManager_setInjectorRate(injectorManager *mgr, const char *injectorName, inject_rate_t rate);
int injectorManager_setInjectorChannel(injectorManager *mgr, const char *injectorName, uint8_t channel);
int injectorManager_setInjectorPower(injectorManager *mgr, const char *injectorName, int8_t tx_power_dbm);
int injectorManager_setInjectorIntervalNs(injectorManager *mgr, const char *injectorName, uint32_t interval_ns);
int injectorManager_setInjectorMaxPackets(injectorManager *mgr, const char *injectorName, uint32_t maxPackets);
int injectorManager_setInjectorRetries(injectorManager *mgr, const char *injectorName, uint8_t maxRetries);
int injectorManager_setInjectorFlags(injectorManager *mgr, const char *injectorName, inject_flags_t flags);
int injectorManager_setInjectorPacketData(injectorManager *mgr, const char *injectorName,
                                          const uint8_t *newData, uint32_t newLen);

int injectorManager_getInjectorInfo(injectorManager *mgr, const char *injectorName, InjectorInfo *info);
int injectorManager_listInjectors(injectorManager *mgr, char names[][INJECTOR_NAME_MAX], int maxCount);

int injectorManager_schedule(injectorManager *mgr, uint64_t now_ns);

#ifdef INJECT_USE_RTOS
int injectorManager_startSchedulerTask(injectorManager *mgr, UBaseType_t priority);
#endif

uint64_t platform_get_time_ns(void);
int platform_set_timer_ns(uint64_t abs_time_ns);

#ifdef __cplusplus
}
#endif

#endif
