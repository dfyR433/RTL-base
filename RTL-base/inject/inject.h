/**
 * @file  inject.h
 * @brief Wi-Fi Packet Injector — RTL8721Dx / KM4 (Cortex-M55)
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

/* =========================================================================
 * Error codes
 * ========================================================================= */
#define INJ_OK               ( 0)
#define INJ_ERR              (-1)
#define INJ_ERR_NOT_FOUND    (-2)
#define INJ_ERR_INVALID_ARG  (-3)
#define INJ_ERR_NO_SPACE     (-4)
#define INJ_ERR_TIMER        (-5)
#define INJ_ERR_CHANNEL      (-6)
#define INJ_ERR_BUSY         (-7)
#define INJ_ERR_RATE         (-8)
#define INJ_ERR_POWER        (-9)
#define INJ_ERR_STATE        (-10)

/* =========================================================================
 * Configuration
 * ========================================================================= */
#define INJECTOR_NAME_MAX              32u
#define INJECTOR_MAX                   16u
#define INJECTOR_MAX_PACKET_SIZE       2048u
#define INJECTOR_SCHED_STACK_WORDS     1536u
#define INJECTOR_STOP_TIMEOUT_MS       1000u
#define INJECTOR_SPIN_GUARD_MIN_NS     50000ULL
#define INJECTOR_SPIN_GUARD_MAX_NS     800000ULL
#define INJECTOR_DEADLINE_WINDOW_NS    50000ULL
#define INJECTOR_BACKOFF_BASE_NS       1000000ULL
#define INJECTOR_BACKOFF_MAX_STEPS     6u
#define INJECTOR_HIST_BINS             8u
#define INJECTOR_HIST_BIN_WIDTH_NS     50000u

/* =========================================================================
 * TX rates
 * ========================================================================= */
typedef enum {
    INJ_RATE_1M   =0x02, INJ_RATE_2M   =0x04, INJ_RATE_5_5M=0x0B,
    INJ_RATE_11M  =0x16, INJ_RATE_6M   =0x0C, INJ_RATE_9M  =0x12,
    INJ_RATE_12M  =0x18, INJ_RATE_18M  =0x24, INJ_RATE_24M =0x30,
    INJ_RATE_36M  =0x48, INJ_RATE_48M  =0x60, INJ_RATE_54M =0x6C,
    INJ_RATE_MCS0 =0x80, INJ_RATE_MCS1 =0x81, INJ_RATE_MCS2=0x82,
    INJ_RATE_MCS3 =0x83, INJ_RATE_MCS4 =0x84, INJ_RATE_MCS5=0x85,
    INJ_RATE_MCS6 =0x86, INJ_RATE_MCS7 =0x87,
    INJ_RATE_DEFAULT = 0x02
} inject_rate_t;

/* =========================================================================
 * Behaviour flags
 * ========================================================================= */
typedef enum {
    INJ_FLAG_NONE           = 0,
    INJ_FLAG_NO_ACK         = (1 << 0),
    INJ_FLAG_USE_SHORT_GI   = (1 << 1),
    INJ_FLAG_AGGREGATE      = (1 << 2),
    INJ_FLAG_FIXED_CHANNEL  = (1 << 3),
    INJ_FLAG_AUTO_SEQ       = (1 << 4),
    INJ_FLAG_SKIP_IF_LATE   = (1 << 5),
    INJ_FLAG_PREDICT_SWITCH = (1 << 6),
    INJ_FLAG_EXP_BACKOFF    = (1 << 7),
} inject_flags_t;

typedef enum { INJ_AC_BE=0, INJ_AC_BK=1, INJ_AC_VI=2, INJ_AC_VO=3 } inject_ac_t;

/* =========================================================================
 * Scheduler task lifecycle states (internal — exposed for diagnostics)
 * ========================================================================= */
typedef enum {
    INJ_SCHED_IDLE    = 0,  /* task not created                              */
    INJ_SCHED_RUNNING = 1,  /* task running normally                         */
    INJ_SCHED_STOPPING= 2,  /* stop requested; task will exit at next check  */
    INJ_SCHED_STOPPED = 3,  /* task exited cleanly and signalled done_sem    */
    INJ_SCHED_STUCK   = 4,  /* stop timed out — task is zombie, log bug      */
} inj_sched_state_t;

/* =========================================================================
 * Timing statistics
 * ========================================================================= */
typedef struct {
    int64_t  jitter_mean_ns;
    uint64_t jitter_var_ns2;
    uint64_t jitter_stddev_ns;
    int64_t  jitter_min_ns;
    int64_t  jitter_max_ns;
    uint64_t switch_mean_ns;
    uint32_t deadline_misses;
    uint32_t deadline_skips;
    uint32_t burst_completions;
    uint32_t jitter_histogram[INJECTOR_HIST_BINS];
    uint32_t tx_air_time_us;
} InjectorTimingStats;

/* =========================================================================
 * Public info snapshot
 * ========================================================================= */
typedef struct {
    char           name[INJECTOR_NAME_MAX];
    bool           active;
    uint8_t        channel;
    uint8_t        priority;
    uint64_t       interval_ns;
    uint32_t       burst_count;
    uint32_t       maxPackets;
    uint32_t       packets_sent;
    uint32_t       tx_errors;
    uint32_t       tx_retries;
    uint8_t        maxHwRetries;
    inject_rate_t  tx_rate;
    int8_t         tx_power_dbm;
    inject_flags_t flags;
    inject_ac_t    ac_queue;
    uint32_t       packetLen;
} InjectorInfo;

typedef void (*inj_mutate_cb_t)(uint8_t *frame, uint32_t len,
                                 uint32_t seq, void *ctx);

typedef struct injectorManager injectorManager;

/* =========================================================================
 * Lifecycle
 * ========================================================================= */
injectorManager *injectorManager_create(void);

void             injectorManager_destroy(injectorManager *mgr);
void             injectorManager_clearAll(injectorManager *mgr);
void             injectorManager_setWlanIndex(injectorManager *mgr, uint8_t idx);
void             injectorManager_setPowerMappingCallback(injectorManager *mgr,
                                                          uint8_t (*cb)(int8_t dbm));

/* =========================================================================
 * Injector management
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
                                   uint8_t          priority);

static inline int injectorManager_setInjector(injectorManager *mgr,
                                               const char      *name,
                                               const uint8_t   *packetData,
                                               uint32_t         packetLen,
                                               uint8_t          channel,
                                               uint64_t         interval_ns,
                                               uint32_t         maxPackets,
                                               uint8_t          hwRetries) {
    return injectorManager_setInjectorEx(mgr, name, packetData, packetLen,
        channel, 0ULL, interval_ns, 1u, maxPackets, hwRetries, 3u,
        INJ_RATE_DEFAULT, -1,
        INJ_FLAG_SKIP_IF_LATE | INJ_FLAG_EXP_BACKOFF,
        INJ_AC_BE, 0u);
}

int injectorManager_deleteInjector    (injectorManager *mgr, const char *name);
int injectorManager_activateInjector  (injectorManager *mgr, const char *name);
int injectorManager_deactivateInjector(injectorManager *mgr, const char *name);

/* =========================================================================
 * Dynamic updates
 * ========================================================================= */
int injectorManager_setRate          (injectorManager *, const char *, inject_rate_t);
int injectorManager_setChannel       (injectorManager *, const char *, uint8_t);
int injectorManager_setTxPower       (injectorManager *, const char *, int8_t);
int injectorManager_setIntervalNs    (injectorManager *, const char *, uint64_t);
int injectorManager_setBurstCount    (injectorManager *, const char *, uint32_t);
int injectorManager_setMaxPackets    (injectorManager *, const char *, uint32_t);
int injectorManager_setHwRetries     (injectorManager *, const char *, uint8_t);
int injectorManager_setSwRetries     (injectorManager *, const char *, uint8_t);
int injectorManager_setFlags         (injectorManager *, const char *, inject_flags_t);
int injectorManager_setAcQueue       (injectorManager *, const char *, inject_ac_t);
int injectorManager_setPriority      (injectorManager *, const char *, uint8_t);
int injectorManager_setPacketData    (injectorManager *, const char *,
                                      const uint8_t *data, uint32_t len);
int injectorManager_setMutateCallback(injectorManager *, const char *,
                                      inj_mutate_cb_t cb, void *ctx);

/* =========================================================================
 * Statistics and diagnostics
 * ========================================================================= */
int              injectorManager_getInfo        (injectorManager *, const char *, InjectorInfo *);
int              injectorManager_getTimingStats (injectorManager *, const char *, InjectorTimingStats *);
int              injectorManager_listInjectors  (injectorManager *,
                                                 char names[][INJECTOR_NAME_MAX], int maxCount);
uint64_t         injectorManager_getTotalPackets(injectorManager *);
uint32_t         injectorManager_getActiveCount (injectorManager *);
uint32_t         injectorManager_getTotalErrors (injectorManager *);
int              injectorManager_resetStats     (injectorManager *, const char *);
int              injectorManager_resetTimingStats(injectorManager *, const char *);

inj_sched_state_t injectorManager_getSchedState(injectorManager *mgr);
int injectorManager_resetStuck(injectorManager *mgr);

uint32_t injectorManager_estimateAirTimeUs(inject_rate_t rate, uint32_t frame_bytes);

/* =========================================================================
 * Scheduler control
 * ========================================================================= */
int injectorManager_startSchedulerTask  (injectorManager *, UBaseType_t priority);
int injectorManager_startSchedulerTaskEx(injectorManager *, UBaseType_t priority,
                                          uint32_t stackWords);
int injectorManager_stopSchedulerTask   (injectorManager *);

/* =========================================================================
 * Logging
 * ========================================================================= */
typedef void (*inj_log_cb_t)(const char *msg);
void injectorManager_setLogCallback(inj_log_cb_t cb);

#ifdef __cplusplus
}
#endif
#endif /* INJECT_H */
