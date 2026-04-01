/**
 * @file inject.h
 * @brief Wi-Fi Packet Injector Manager – high-precision scheduling
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
 *                           Configuration Macros
 *============================================================================*/
#define INJECTOR_NAME_MAX         32
#define INJECTOR_MAX              16
#define INJECTOR_MAX_PACKET_SIZE  2048
#define INJECTOR_STOP_TIMEOUT_MS  200
#define INJECTOR_SPIN_GUARD_NS    2000000ULL

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
    INJ_FLAG_NONE           = 0,
    INJ_FLAG_NO_ACK         = (1 << 0),
    INJ_FLAG_USE_SHORT_GI   = (1 << 1),
    INJ_FLAG_AGGREGATE      = (1 << 2),
    INJ_FLAG_FIXED_CHANNEL  = (1 << 3),
} inject_flags_t;

typedef enum {
    INJ_AC_BE = 0,  /**< Best effort  */
    INJ_AC_BK = 1,  /**< Background   */
    INJ_AC_VI = 2,  /**< Video        */
    INJ_AC_VO = 3   /**< Voice        */
} inject_ac_t;

typedef struct {
    char           name[INJECTOR_NAME_MAX];
    bool           active;
    uint8_t        channel;
    uint64_t       interval_ns;
    uint32_t       maxPackets;
    uint32_t       packets_sent;
    uint32_t       tx_errors;
    uint32_t       tx_retries;
    uint8_t        maxRetries;
    inject_rate_t  tx_rate;
    int8_t         tx_power_dbm;
    inject_flags_t flags;
    inject_ac_t    ac_queue;
    uint32_t       packetLen;
} InjectorInfo;

typedef struct injectorManager injectorManager;

/*============================================================================
 *                           Public Functions
 *============================================================================*/

/* ── Lifecycle ─────────────────────────────────────────────────────────── */
void injector_set_timer_freq_hz(uint32_t freq);

injectorManager *injectorManager_create(void);

void injectorManager_destroy(injectorManager *mgr);
void injectorManager_clearAll(injectorManager *mgr);

void injectorManager_setWlanIndex(injectorManager *mgr, uint8_t wlan_idx);
void injectorManager_setPowerMappingCallback(injectorManager *mgr,
                                             uint8_t (*callback)(int8_t dbm));

/* ── Injector management ───────────────────────────────────────────────── */

/**
 * @brief Create or reconfigure an injector (left inactive).
 *
 * Reconfiguring an existing injector by name resets all statistics and
 * timing-compensation accumulators.  The injector must be activated
 * separately with injectorManager_activateInjector().
 *
 * @param mgr           Manager instance.
 * @param name          Unique name (max INJECTOR_NAME_MAX-1 chars).
 * @param packetData    Frame payload (copied internally).
 * @param packetLen     Payload length (max INJECTOR_MAX_PACKET_SIZE).
 * @param channel       Wi-Fi channel: 0 = use current; 1-13 (2.4 GHz);
 *                      36-165 (5 GHz). Returns INJ_ERR_CHANNEL if invalid.
 * @param start_time_ns Absolute start time (ns). 0 = start immediately.
 * @param interval_ns   Interval between transmissions (ns).
 *                      Pass 0 for single-shot (one packet, then done).
 * @param maxPackets    Packet limit; 0 = unlimited. Ignored if interval_ns==0.
 * @param hwRetries     Hardware retry count passed to the Wi-Fi driver.
 * @param swRetries     Software retry attempts when hardware fails.
 * @param rate          Transmit rate (inject_rate_t). Returns INJ_ERR_RATE if
 *                      the value is not a member of inject_rate_t.
 * @param tx_power_dbm  TX power in dBm. -1 = use current/default.
 * @param flags         Behaviour flags (inject_flags_t).
 * @param ac_queue      WMM access category.
 * @return INJ_OK or error code.
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

/* ── Dynamic parameter updates ─────────────────────────────────────────── */
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

/* ── Statistics and information ─────────────────────────────────────────── */
int      injectorManager_getInfo(injectorManager *mgr, const char *name, InjectorInfo *info);
int      injectorManager_listInjectors(injectorManager *mgr,
                                       char names[][INJECTOR_NAME_MAX],
                                       int maxCount);
uint64_t injectorManager_getTotalPackets(injectorManager *mgr);
uint32_t injectorManager_getActiveCount(injectorManager *mgr);
uint32_t injectorManager_getTotalErrors(injectorManager *mgr);

int injectorManager_resetStats(injectorManager *mgr, const char *name);

/* ── Scheduler control ──────────────────────────────────────────────────── */
int injectorManager_startSchedulerTask(injectorManager *mgr, UBaseType_t priority);
int injectorManager_startSchedulerTaskEx(injectorManager *mgr, UBaseType_t priority,
                                         uint32_t stackSize);
int injectorManager_stopSchedulerTask(injectorManager *mgr);

/* ── Platform abstraction (must be provided by the application) ─────────── */
uint64_t platform_get_time_ns(void);

/* ── Optional logging callback ──────────────────────────────────────────── */
typedef void (*inj_log_cb_t)(const char *msg);
void injectorManager_setLogCallback(inj_log_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* INJECT_H */
