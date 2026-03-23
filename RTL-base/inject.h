/**
 * @file inject.h
 * @brief Wi‑Fi Packet Injector Manager
 */

#ifndef INJECT_H
#define INJECT_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 *                              Error Codes
 *============================================================================*/
#define INJ_OK                (0)
#define INJ_ERR               (-1)
#define INJ_ERR_NOT_FOUND     (-2)
#define INJ_ERR_INVALID_ARG   (-3)
#define INJ_ERR_NO_SPACE      (-4)
#define INJ_ERR_TIMER         (-5)
#define INJ_ERR_CHANNEL       (-6)
#define INJ_ERR_BUSY          (-7)
#define INJ_ERR_RATE          (-8)
#define INJ_ERR_POWER         (-9)
#define INJ_ERR_STATE         (-10)

/*============================================================================
 *                            Configuration
 *============================================================================*/
#define INJECTOR_NAME_MAX     32
#define INJECTOR_MAX          16

/*============================================================================
 *                              Data Types
 *============================================================================*/
typedef enum {
    INJ_RATE_1M   = 0x02,
    INJ_RATE_2M   = 0x04,
    INJ_RATE_5_5M = 0x0B,
    INJ_RATE_11M  = 0x16,
    INJ_RATE_6M   = 0x0C,
    INJ_RATE_9M   = 0x12,
    INJ_RATE_12M  = 0x18,
    INJ_RATE_18M  = 0x24,
    INJ_RATE_24M  = 0x30,
    INJ_RATE_36M  = 0x48,
    INJ_RATE_48M  = 0x60,
    INJ_RATE_54M  = 0x6C,
    INJ_RATE_MCS0 = 0x80,
    INJ_RATE_MCS1 = 0x81,
    INJ_RATE_MCS2 = 0x82,
    INJ_RATE_MCS3 = 0x83,
    INJ_RATE_MCS4 = 0x84,
    INJ_RATE_MCS5 = 0x85,
    INJ_RATE_MCS6 = 0x86,
    INJ_RATE_MCS7 = 0x87,
    INJ_RATE_DEFAULT = INJ_RATE_1M
} inject_rate_t;

typedef enum {
    INJ_FLAG_NONE          = 0,
    INJ_FLAG_NO_ACK        = (1 << 0),
    INJ_FLAG_USE_SHORT_GI  = (1 << 1),
    INJ_FLAG_AGGREGATE     = (1 << 2),
    INJ_FLAG_FIXED_CHANNEL = (1 << 3),
} inject_flags_t;

typedef enum {
    INJ_AC_BE = 0,
    INJ_AC_BK = 1,
    INJ_AC_VI = 2,
    INJ_AC_VO = 3
} inject_ac_t;

typedef struct {
    char name[INJECTOR_NAME_MAX];
    bool active;
    uint8_t channel;
    uint64_t interval_ns;
    uint32_t maxPackets;
    uint32_t packets_sent;
    uint8_t  maxRetries;
    uint32_t tx_errors;
    inject_rate_t tx_rate;
    int8_t   tx_power_dbm;
    inject_flags_t flags;
    inject_ac_t ac_queue;
    uint32_t packetLen;
} InjectorInfo;

typedef struct injectorManager injectorManager;

/*============================================================================
 *                           Public Functions
 *============================================================================*/
injectorManager* injectorManager_create(void);
void injectorManager_destroy(injectorManager *mgr);
void injectorManager_clearAll(injectorManager *mgr);

/**
 * @brief Set up an injector (inactive) with nanosecond timing.
 * @param mgr           Manager instance
 * @param name          Unique injector name
 * @param packetData    Packet data (copied internally)
 * @param packetLen     Length of packet data
 * @param channel       Wi-Fi channel (1-13, 36-165)
 * @param start_time_ns Absolute start time (nanoseconds). 0 = now.
 * @param interval_ns   Interval between transmissions (nanoseconds)
 * @param maxPackets    Maximum number of packets (0 = unlimited)
 * @param hwRetries     Hardware retry limit
 * @param swRetries     Software retry limit
 * @param rate          Transmit rate
 * @param tx_power_dbm  Transmit power (dBm), -1 = default
 * @param flags         Behaviour flags
 * @param ac_queue      Access category queue
 * @return INJ_OK or error code
 */
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
                                  inject_ac_t ac_queue);

/**
 * @brief injector creation (default parameters).
 */
static inline int injectorManager_setInjector(injectorManager *mgr,
                                              const char *name,
                                              const uint8_t *packetData,
                                              uint32_t packetLen,
                                              uint8_t channel,
                                              uint64_t interval_ns,
                                              uint32_t maxPackets,
                                              uint8_t hwRetries) {
    return injectorManager_setInjectorEx(mgr, name, packetData, packetLen,
                                         channel, 0, interval_ns,
                                         maxPackets, hwRetries, 5,
                                         INJ_RATE_DEFAULT, -1,
                                         INJ_FLAG_NONE, INJ_AC_BE);
}

int injectorManager_deleteInjector(injectorManager *mgr, const char *name);
int injectorManager_activateInjector(injectorManager *mgr, const char *name);
int injectorManager_deactivateInjector(injectorManager *mgr, const char *name);

/* Dynamic parameter updates */
int injectorManager_setRate(injectorManager *mgr, const char *name, inject_rate_t rate);
int injectorManager_setChannel(injectorManager *mgr, const char *name, uint8_t channel);
int injectorManager_setTxPower(injectorManager *mgr, const char *name, int8_t tx_power_dbm);
int injectorManager_setIntervalNs(injectorManager *mgr, const char *name, uint64_t interval_ns);
int injectorManager_setMaxPackets(injectorManager *mgr, const char *name, uint32_t maxPackets);
int injectorManager_setHwRetries(injectorManager *mgr, const char *name, uint8_t hwRetries);
int injectorManager_setSwRetries(injectorManager *mgr, const char *name, uint8_t swRetries);
int injectorManager_setFlags(injectorManager *mgr, const char *name, inject_flags_t flags);
int injectorManager_setAcQueue(injectorManager *mgr, const char *name, inject_ac_t ac_queue);
int injectorManager_setPacketData(injectorManager *mgr, const char *name,
                                  const uint8_t *newData, uint32_t newLen);

/* Information and status */
int injectorManager_getInfo(injectorManager *mgr, const char *name, InjectorInfo *info);
int injectorManager_listInjectors(injectorManager *mgr,
                                  char names[][INJECTOR_NAME_MAX],
                                  int maxCount);
uint64_t injectorManager_getTotalPackets(injectorManager *mgr);
uint32_t injectorManager_getActiveCount(injectorManager *mgr);

/* Scheduling */
int injectorManager_startSchedulerTask(injectorManager *mgr, UBaseType_t priority);

/* Platform abstraction – must be provided by the application */
uint64_t platform_get_time_ns(void);

#ifdef __cplusplus
}
#endif

#endif /* INJECT_H */
