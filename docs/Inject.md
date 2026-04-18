# Injector Library — `inject.h` / `inject.c`

**Target:** RTL8721Dx (Ameba) — KM4 core (ARM Cortex-M55)  
**Purpose:** Precision-scheduled, multi-stream 802.11 raw frame injection over the Realtek WiFi driver  
**Thread safety:** All `injectorManager_*` API functions are mutex-protected and task-safe. The scheduler task runs independently and accesses shared state through atomic operations and the same mutex.

---

## Overview

The injector library manages up to `INJECTOR_MAX` (16) independent logical injectors, each representing a named, parameterized stream of raw 802.11 frames. A single shared FreeRTOS scheduler task dispatches all active injectors using an **EDF (Earliest Deadline First) binary min-heap**, sleeping between deadlines and waking precisely at the next scheduled transmission.

### Key architectural decisions

**Phase-accumulator scheduling.** Each injector's next deadline is computed as:

```
next_deadline = start_time_ns + burst_index × interval_ns
```

This avoids cumulative drift from naive interval addition (`deadline += interval`) over thousands of bursts. Long-term accuracy depends only on the timer library's composite timestamp quality.

**Double-buffered packet data.** Each injector maintains two packet buffers (`buf[0]` / `buf[1]`) with an atomic index that the scheduler reads while the application can safely write to the inactive buffer via `injectorManager_setPacketData()` and `injectorManager_setMutateCallback()`. No blocking occurs on the injection hot path.

**Per-injector channel isolation.** The single RTL8721Dx radio is shared across all injectors. When an injector's target channel differs from the current radio channel, the scheduler performs a channel switch with a configurable settle-time budget. Injectors with `INJ_FLAG_FIXED_CHANNEL` are always served from their channel; others may tolerate a missed deadline if the switch budget is exceeded.

**EDF heap + lookahead pre-switch.** The scheduler can look one slot ahead in the heap to pre-switch the radio channel before the next injector's deadline arrives, hiding switch latency inside the current injector's inter-burst gap.

---

## Configuration Macros

| Macro | Default | Description |
|-------|---------|-------------|
| `INJECTOR_NAME_MAX` | `32` | Maximum injector name length including null terminator |
| `INJECTOR_MAX` | `16` | Maximum simultaneous injectors |
| `INJECTOR_MAX_PACKET_SIZE` | `2048` | Maximum raw frame size in bytes |
| `INJECTOR_SCHED_STACK_WORDS` | `1536` | Default scheduler task stack in 32-bit words |
| `INJECTOR_STOP_TIMEOUT_MS` | `1000` | Maximum time to wait for the scheduler task to exit cleanly after `stopSchedulerTask()` |
| `INJECTOR_SPIN_GUARD_MIN_NS` | `50,000` | Minimum spin-wait guard window before a deadline (50 µs). Deadlines within this window trigger immediate TX. |
| `INJECTOR_SPIN_GUARD_MAX_NS` | `800,000` | Maximum spin time before a deadline (800 µs). Gaps larger than this use `vTaskDelay`. |
| `INJECTOR_DEADLINE_WINDOW_NS` | `50,000` | A missed deadline is declared only if actual TX time exceeds deadline + this window (50 µs tolerance). |
| `INJECTOR_BACKOFF_BASE_NS` | `1,000,000` | Base interval for exponential backoff on TX error (1 ms). |
| `INJECTOR_BACKOFF_MAX_STEPS` | `6` | Maximum backoff exponent: max delay = `base × 2^6 = 64 ms`. |
| `INJECTOR_HIST_BINS` | `8` | Jitter histogram bin count. |
| `INJECTOR_HIST_BIN_WIDTH_NS` | `50,000` | Jitter histogram bin width (50 µs). |

---

## Types

### `inject_rate_t` — TX PHY rate

```c
typedef enum {
    INJ_RATE_1M    = 0x02,   // 802.11b DSSS
    INJ_RATE_2M    = 0x04,
    INJ_RATE_5_5M  = 0x0B,
    INJ_RATE_11M   = 0x16,
    INJ_RATE_6M    = 0x0C,   // 802.11g/a OFDM
    INJ_RATE_9M    = 0x12,
    INJ_RATE_12M   = 0x18,
    INJ_RATE_18M   = 0x24,
    INJ_RATE_24M   = 0x30,
    INJ_RATE_36M   = 0x48,
    INJ_RATE_48M   = 0x60,
    INJ_RATE_54M   = 0x6C,
    INJ_RATE_MCS0  = 0x80,   // 802.11n HT
    // ... MCS1–MCS7 = 0x81–0x87
    INJ_RATE_DEFAULT = 0x02  // 1 Mbps
} inject_rate_t;
```

Values map directly to Realtek SDK `MGN_RATE` constants passed to the driver's `raw_frame_desc_t`.

---

### `inject_flags_t` — Behaviour flags

| Flag | Bit | Description |
|------|-----|-------------|
| `INJ_FLAG_NONE` | — | No flags |
| `INJ_FLAG_NO_ACK` | 0 | Transmit without waiting for 802.11 ACK (sets `NoACK` in the frame descriptor). |
| `INJ_FLAG_USE_SHORT_GI` | 1 | Use 400 ns short guard interval for HT rates (MCS0–MCS7). Ignored for OFDM/DSSS. |
| `INJ_FLAG_AGGREGATE` | 2 | Enable A-MPDU aggregation hint to the driver. |
| `INJ_FLAG_FIXED_CHANNEL` | 3 | Do not switch channel for this injector. If the radio is on a different channel, the burst is either delayed or skipped depending on `INJ_FLAG_SKIP_IF_LATE`. |
| `INJ_FLAG_AUTO_SEQ` | 4 | Auto-increment the 802.11 Sequence Control field (bits [15:4] of the Sequence Control word). The library patches the frame buffer before each TX. |
| `INJ_FLAG_SKIP_IF_LATE` | 5 | If the scheduler wakes up past a deadline by more than `INJECTOR_DEADLINE_WINDOW_NS`, skip that burst rather than transmitting out-of-phase. Increments `deadline_skips`. |
| `INJ_FLAG_PREDICT_SWITCH` | 6 | Enable lookahead pre-switching: after dispatching the current burst, peek at the next injector in the heap and pre-switch the radio if the channel differs and the gap is large enough to absorb settle time. |
| `INJ_FLAG_EXP_BACKOFF` | 7 | On TX error, back off exponentially (up to `INJECTOR_BACKOFF_MAX_STEPS` steps) before the next attempt. Increments `tx_retries`. |

---

### `inject_ac_t` — WMM Access Category

```c
typedef enum {
    INJ_AC_BE = 0,  // Best Effort  (EDCA AC_BE)
    INJ_AC_BK = 1,  // Background   (EDCA AC_BK)
    INJ_AC_VI = 2,  // Video        (EDCA AC_VI)
    INJ_AC_VO = 3,  // Voice        (EDCA AC_VO)
} inject_ac_t;
```

Maps to the WMM queue index in `raw_frame_desc_t`. Higher priority queues have shorter AIFS and smaller CW, providing statistical priority in contention-based access.

---

### `inj_sched_state_t` — Scheduler lifecycle state

| State | Value | Meaning |
|-------|-------|---------|
| `INJ_SCHED_IDLE` | 0 | Task not yet created |
| `INJ_SCHED_RUNNING` | 1 | Task running normally |
| `INJ_SCHED_STOPPING` | 2 | `stopSchedulerTask()` called; task will exit at next iteration |
| `INJ_SCHED_STOPPED` | 3 | Task exited cleanly and posted to `done_sem` |
| `INJ_SCHED_STUCK` | 4 | `stopSchedulerTask()` timed out — task is a zombie. Call `injectorManager_resetStuck()`. |

---

### `InjectorTimingStats`

Per-injector timing statistics snapshot, populated by `injectorManager_getTimingStats()`.

| Field | Type | Description |
|-------|------|-------------|
| `jitter_mean_ns` | `int64_t` | EMA of signed jitter (actual TX time − scheduled deadline), in nanoseconds |
| `jitter_var_ns2` | `uint64_t` | EMA of jitter variance (ns²) |
| `jitter_stddev_ns` | `uint64_t` | `sqrt(jitter_var_ns2)`, pre-computed |
| `jitter_min_ns` | `int64_t` | Minimum observed jitter (most early TX) |
| `jitter_max_ns` | `int64_t` | Maximum observed jitter (most late TX) |
| `switch_mean_ns` | `uint64_t` | EMA of channel switch settle time, in nanoseconds |
| `deadline_misses` | `uint32_t` | Total deadline misses (TX happened, but late by > `INJECTOR_DEADLINE_WINDOW_NS`) |
| `deadline_skips` | `uint32_t` | Total bursts skipped due to `INJ_FLAG_SKIP_IF_LATE` |
| `burst_completions` | `uint32_t` | Total burst cycles completed successfully |
| `jitter_histogram[8]` | `uint32_t[]` | Jitter distribution: bin `i` covers `[i×50µs, (i+1)×50µs)`. Bin 7 is the overflow bin. |
| `tx_air_time_us` | `uint32_t` | Cumulative estimated on-air TX time in microseconds (sum of `estimateAirTimeUs()` per burst) |

---

### `InjectorInfo`

Read-only configuration snapshot, populated by `injectorManager_getInfo()`.

| Field | Type | Description |
|-------|------|-------------|
| `name` | `char[32]` | Injector name |
| `active` | `bool` | Whether the injector is currently scheduled |
| `channel` | `uint8_t` | Target 802.11 channel |
| `priority` | `uint8_t` | EDF heap priority tiebreaker (lower value = higher priority) |
| `interval_ns` | `uint64_t` | Inter-burst interval |
| `burst_count` | `uint32_t` | Frames per burst |
| `maxPackets` | `uint32_t` | Total burst cycles before auto-deactivation (0 = unlimited) |
| `packets_sent` | `uint32_t` | Cumulative burst cycles dispatched |
| `tx_errors` | `uint32_t` | Driver TX errors |
| `tx_retries` | `uint32_t` | Software retry attempts |
| `maxHwRetries` | `uint8_t` | Hardware retry count passed to driver |
| `tx_rate` | `inject_rate_t` | PHY rate |
| `tx_power_dbm` | `int8_t` | TX power (-1 = driver default) |
| `flags` | `inject_flags_t` | Behaviour flags |
| `ac_queue` | `inject_ac_t` | WMM queue |
| `packetLen` | `uint32_t` | Active packet buffer length |

---

## API Reference

### Lifecycle

#### `injectorManager *injectorManager_create(void)`

Allocates and initializes an `injectorManager` instance. Returns `NULL` on heap allocation failure. Each application typically creates one manager for the lifetime of the program.

---

#### `void injectorManager_destroy(injectorManager *mgr)`

Stops the scheduler task (if running), deactivates all injectors, and frees all associated memory. Safe to call from any task context. Do not use `mgr` after this call.

---

#### `void injectorManager_clearAll(injectorManager *mgr)`

Deactivates and removes all injectors without destroying the manager or stopping the scheduler. Equivalent to calling `deleteInjector()` for every registered injector.

---

#### `void injectorManager_setWlanIndex(injectorManager *mgr, uint8_t idx)`

Sets the WiFi interface index used for all TX operations (default: `STA_WLAN_INDEX`). Must be called before `startSchedulerTask()` if using a non-default interface.

---

#### `void injectorManager_setPowerMappingCallback(injectorManager *mgr, uint8_t (*cb)(int8_t dbm))`

Registers a callback that converts a signed dBm value to the driver's internal power level encoding. If not set, TX power control is disabled and `tx_power_dbm = -1` is treated as "driver default."

---

### Injector Management

#### `int injectorManager_setInjectorEx(mgr, name, packetData, packetLen, channel, start_time_ns, interval_ns, burst_count, maxPackets, hwRetries, swRetries, rate, tx_power_dbm, flags, ac_queue, priority)`

Creates or updates an injector with full parameter control.

| Parameter | Type | Description |
|-----------|------|-------------|
| `name` | `const char *` | Unique injector identifier. Max `INJECTOR_NAME_MAX - 1` chars. |
| `packetData` | `const uint8_t *` | Raw 802.11 frame bytes. Copied into the internal double buffer. |
| `packetLen` | `uint32_t` | Frame length. Must be ≤ `INJECTOR_MAX_PACKET_SIZE`. |
| `channel` | `uint8_t` | Target 802.11 channel (1–14 for 2.4 GHz, 36–165 for 5 GHz). |
| `start_time_ns` | `uint64_t` | Absolute start time from `timer_get_time_ns_fine()`. Pass `0` to start immediately. |
| `interval_ns` | `uint64_t` | Inter-burst interval in nanoseconds. |
| `burst_count` | `uint32_t` | Number of frames in each burst. |
| `maxPackets` | `uint32_t` | Total burst cycles. `0` = run indefinitely. |
| `hwRetries` | `uint8_t` | 802.11 hardware retry limit passed to the driver. |
| `swRetries` | `uint8_t` | Software retry count on driver TX error before recording `tx_errors`. |
| `rate` | `inject_rate_t` | PHY TX rate. |
| `tx_power_dbm` | `int8_t` | TX power. `-1` = driver default. Requires power mapping callback. |
| `flags` | `inject_flags_t` | Bitmask of `INJ_FLAG_*` constants. |
| `ac_queue` | `inject_ac_t` | WMM queue. |
| `priority` | `uint8_t` | EDF tiebreaker. Lower = higher priority when deadlines are equal. |

**Returns:** `INJ_OK` on success. If an injector with `name` already exists, it is updated in-place (the scheduler is not restarted). Returns `INJ_ERR_NO_SPACE` if `INJECTOR_MAX` injectors are already registered, `INJ_ERR_INVALID_ARG` for out-of-range parameters.

---

#### `int injectorManager_setInjector(mgr, name, packetData, packetLen, channel, interval_ns, maxPackets, hwRetries)`

Simplified wrapper over `setInjectorEx` with sensible defaults:

- `start_time_ns = 0` (immediate)
- `burst_count = 1`
- `swRetries = 3`
- `rate = INJ_RATE_DEFAULT` (1 Mbps)
- `tx_power_dbm = -1` (driver default)
- `flags = INJ_FLAG_SKIP_IF_LATE | INJ_FLAG_EXP_BACKOFF`
- `ac_queue = INJ_AC_BE`
- `priority = 0`

---

#### `int injectorManager_deleteInjector(injectorManager *mgr, const char *name)`

Removes an injector. If it is active, it is deactivated first. The scheduler heap is rebuilt. Returns `INJ_ERR_NOT_FOUND` if the name is not registered.

---

#### `int injectorManager_activateInjector(injectorManager *mgr, const char *name)`

Marks an injector as active and inserts it into the EDF heap. The injector will be dispatched by the scheduler task on its next deadline. Returns `INJ_ERR_STATE` if the scheduler task is not running.

---

#### `int injectorManager_deactivateInjector(injectorManager *mgr, const char *name)`

Removes an injector from the EDF heap without deleting it. The injector's configuration and statistics are preserved. Can be reactivated later.

---

### Dynamic Updates

All update functions take `(injectorManager *mgr, const char *name, ...)` and return `INJ_OK` or an error code. Updates are applied atomically under the manager mutex.

| Function | Parameter | Notes |
|----------|-----------|-------|
| `setRate` | `inject_rate_t` | Takes effect on the next burst. |
| `setChannel` | `uint8_t` | The scheduler will switch channel on the next dispatch. |
| `setTxPower` | `int8_t` | Requires power mapping callback. |
| `setIntervalNs` | `uint64_t` | Resets phase accumulator to current time to avoid phase discontinuity. |
| `setBurstCount` | `uint32_t` | Takes effect on the next burst cycle. |
| `setMaxPackets` | `uint32_t` | `0` = unlimited. Takes effect immediately. |
| `setHwRetries` | `uint8_t` | Applied to the driver on the next TX. |
| `setSwRetries` | `uint8_t` | Applied on the next TX attempt. |
| `setFlags` | `inject_flags_t` | Replaces the entire flags word. |
| `setAcQueue` | `inject_ac_t` | Takes effect on the next burst. |
| `setPriority` | `uint8_t` | Triggers a heap rebuild. |
| `setPacketData` | `const uint8_t *, uint32_t` | Writes to the inactive double buffer, then atomically flips the active index. No blocking on the scheduler path. |
| `setMutateCallback` | `inj_mutate_cb_t, void *ctx` | Registers a per-burst frame mutation callback (see below). |

---

#### Mutate Callback — `inj_mutate_cb_t`

```c
typedef void (*inj_mutate_cb_t)(uint8_t *frame, uint32_t len,
                                 uint32_t seq, void *ctx);
```

Called from the scheduler task immediately before each TX. `frame` points to the active packet buffer (writable). `seq` is the current burst sequence number (monotonically increasing per injector). `ctx` is the user-supplied pointer.

Use this to patch per-burst fields (e.g., sequence numbers, payload bytes, timestamps embedded in the frame body) without the overhead of a full `setPacketData()` copy. The callback must complete quickly — it executes on the scheduler task's deadline path.

---

### Statistics and Diagnostics

#### `int injectorManager_getInfo(injectorManager *mgr, const char *name, InjectorInfo *out)`

Fills `*out` with a snapshot of the injector's current configuration and counters. Thread-safe. Returns `INJ_ERR_NOT_FOUND` if the name is unknown.

---

#### `int injectorManager_getTimingStats(injectorManager *mgr, const char *name, InjectorTimingStats *out)`

Fills `*out` with the injector's timing statistics. Returns `INJ_ERR_NOT_FOUND` if the name is unknown.

---

#### `int injectorManager_listInjectors(injectorManager *mgr, char names[][INJECTOR_NAME_MAX], int maxCount)`

Writes up to `maxCount` injector names into `names`. Returns the total number of registered injectors (may exceed `maxCount`).

---

#### `uint64_t injectorManager_getTotalPackets(injectorManager *mgr)`

Returns the sum of `packets_sent` across all registered injectors.

---

#### `uint32_t injectorManager_getActiveCount(injectorManager *mgr)`

Returns the number of injectors currently in the EDF heap (active state).

---

#### `uint32_t injectorManager_getTotalErrors(injectorManager *mgr)`

Returns the sum of `tx_errors` across all registered injectors.

---

#### `int injectorManager_resetStats(injectorManager *mgr, const char *name)`

Resets `packets_sent`, `tx_errors`, and `tx_retries` for the named injector.

---

#### `int injectorManager_resetTimingStats(injectorManager *mgr, const char *name)`

Resets all fields in the injector's `InjectorTimingStats`, including the jitter histogram.

---

#### `inj_sched_state_t injectorManager_getSchedState(injectorManager *mgr)`

Returns the current scheduler task lifecycle state without acquiring any lock. Safe to call from any context including ISR (read is atomic on Cortex-M).

---

#### `int injectorManager_resetStuck(injectorManager *mgr)`

If `getSchedState()` returns `INJ_SCHED_STUCK`, this function resets the state to `INJ_SCHED_IDLE` so that `startSchedulerTask()` can be called again. The zombie task remains in the kernel; this is a recovery path for a bug that should not occur in production.

---

#### `uint32_t injectorManager_estimateAirTimeUs(inject_rate_t rate, uint32_t frame_bytes)`

Estimates on-air TX duration in microseconds for a given PHY rate and frame length. Accounts for PLCP preamble and header overhead for the appropriate PHY (DSSS, OFDM, or HT). Does not account for SIFS/DIFS/backoff — this is PPDU air time only.

Used internally to compute `tx_air_time_us` in `InjectorTimingStats`.

---

### Scheduler Control

#### `int injectorManager_startSchedulerTask(injectorManager *mgr, UBaseType_t priority)`

Creates the EDF scheduler FreeRTOS task with the default stack size (`INJECTOR_SCHED_STACK_WORDS`). Returns `INJ_OK` on success, `INJ_ERR_STATE` if the task is already running, or `INJ_ERR` on `xTaskCreate` failure.

---

#### `int injectorManager_startSchedulerTaskEx(injectorManager *mgr, UBaseType_t priority, uint32_t stackWords)`

As above, with an explicit stack size. Use when `INJECTOR_SCHED_STACK_WORDS` is insufficient (e.g., when the mutate callback does significant stack work).

---

#### `int injectorManager_stopSchedulerTask(injectorManager *mgr)`

Sets the scheduler state to `INJ_SCHED_STOPPING` and blocks for up to `INJECTOR_STOP_TIMEOUT_MS` on a binary semaphore posted by the task when it exits. Returns `INJ_OK` on clean exit, `INJ_ERR_BUSY` on timeout (state becomes `INJ_SCHED_STUCK`).

---

### Logging

#### `void injectorManager_setLogCallback(inj_log_cb_t cb)`

Registers a global log callback (`typedef void (*inj_log_cb_t)(const char *msg)`). When set, the library routes internal diagnostic messages (errors, state transitions, missed deadlines) through this callback. Thread-safe: the callback may be called from the scheduler task context.

---

## Flag Interaction Notes

- `INJ_FLAG_FIXED_CHANNEL` and `INJ_FLAG_PREDICT_SWITCH` on the same injector: `FIXED_CHANNEL` wins. The injector never drives a channel switch, but the scheduler will still pre-switch based on *other* injectors ahead in the heap.
- `INJ_FLAG_FIXED_CHANNEL` and `INJ_FLAG_SKIP_IF_LATE` together: if the radio is on a different channel, the burst is skipped immediately (no channel switch, no wait). This is the correct mode for injectors that must not interfere with other radio operations.
- `INJ_FLAG_AUTO_SEQ` patches bytes at the 802.11 Sequence Control field offset (byte 22–23 in a standard management/data frame). Ensure your frame template has a valid Sequence Control field location. Incompatible with frames that embed QoS Control before Sequence Control (non-standard layouts).
- `INJ_FLAG_EXP_BACKOFF` interacts with `INJ_FLAG_SKIP_IF_LATE`: if a burst is deferred due to backoff and the backed-off deadline falls past the next phase-accumulator deadline, the phase accumulator advances anyway (a deadline slip is recorded, not a skip).

---

## Error Codes

| Code | Value | Meaning |
|------|-------|---------|
| `INJ_OK` | 0 | Success |
| `INJ_ERR` | -1 | Generic error |
| `INJ_ERR_NOT_FOUND` | -2 | Named injector not registered |
| `INJ_ERR_INVALID_ARG` | -3 | Parameter out of range or NULL |
| `INJ_ERR_NO_SPACE` | -4 | `INJECTOR_MAX` injectors already registered |
| `INJ_ERR_TIMER` | -5 | Timer library not ready |
| `INJ_ERR_CHANNEL` | -6 | Channel switch failed |
| `INJ_ERR_BUSY` | -7 | Operation blocked (e.g., stop timeout) |
| `INJ_ERR_RATE` | -8 | Invalid TX rate for current PHY mode |
| `INJ_ERR_POWER` | -9 | TX power change failed (no mapping callback or driver error) |
| `INJ_ERR_STATE` | -10 | Scheduler state does not permit the operation |

---

## Usage Example

```c
#include "inject.h"
#include "timer.h"

// 802.11 Probe Request frame template (minimal, channel 6)
static const uint8_t probe_req[] = {
    0x40, 0x00,              // Frame Control: Probe Request
    0x00, 0x00,              // Duration
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,  // DA: broadcast
    0x12,0x34,0x56,0x78,0x9A,0xBC, // SA: our MAC
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,  // BSSID: broadcast
    0x00, 0x00,              // Sequence Control (patched if INJ_FLAG_AUTO_SEQ)
    0x00, 0x00,              // SSID: wildcard
    0x01, 0x08,              // Supported Rates IE
    0x82,0x84,0x8B,0x96,0x0C,0x12,0x24,0x48
};

static void frame_mutator(uint8_t *frame, uint32_t len, uint32_t seq, void *ctx)
{
    (void)ctx;
    // Embed burst sequence number in a vendor-specific field (example)
    if (len >= 40) {
        frame[38] = (uint8_t)(seq >> 8);
        frame[39] = (uint8_t)(seq & 0xFF);
    }
}

void injector_demo(void)
{
    timer_init();
    timer_calibrate();

    injectorManager *mgr = injectorManager_create();
    injectorManager_setWlanIndex(mgr, STA_WLAN_INDEX);

    // Create a probe request stream: 10 ms interval, unlimited, channel 6
    injectorManager_setInjectorEx(
        mgr, "probe_req",
        probe_req, sizeof(probe_req),
        6,                          // channel
        0ULL,                       // start_time_ns: immediate
        10000000ULL,                // interval_ns: 10 ms
        1,                          // burst_count
        0,                          // maxPackets: unlimited
        3,                          // hwRetries
        2,                          // swRetries
        INJ_RATE_6M,                // rate
        -1,                         // tx_power: default
        INJ_FLAG_AUTO_SEQ | INJ_FLAG_SKIP_IF_LATE | INJ_FLAG_EXP_BACKOFF,
        INJ_AC_BE,
        0                           // priority
    );

    injectorManager_setMutateCallback(mgr, "probe_req", frame_mutator, NULL);
    injectorManager_activateInjector(mgr, "probe_req");
    injectorManager_startSchedulerTask(mgr, tskIDLE_PRIORITY + 4u);

    vTaskDelay(pdMS_TO_TICKS(5000));

    // Read back stats
    InjectorTimingStats ts;
    injectorManager_getTimingStats(mgr, "probe_req", &ts);
    printf("Jitter mean: %lld ns, stddev: %llu ns, misses: %lu\n",
           (long long)ts.jitter_mean_ns,
           (unsigned long long)ts.jitter_stddev_ns,
           (unsigned long)ts.deadline_misses);

    injectorManager_stopSchedulerTask(mgr);
    injectorManager_destroy(mgr);
}
```
