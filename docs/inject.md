# Wi-Fi Packet Injector Manager

## Introduction

The **Wi-Fi Packet Injector Manager** is a real-time library for high-precision,
scheduled transmission of raw Wi-Fi frames on embedded systems running FreeRTOS.
It supports multiple named injectors each with its own payload, rate, channel,
power, retry policies, and nanosecond-resolution scheduling.

---

## Features

- **Multiple independent injectors** – up to 16 named injectors (configurable via `INJECTOR_MAX`).
- **High-precision scheduling** – nanosecond-accurate transmission times with sub-ms spin guard.
- **Single-shot and periodic modes** – `interval_ns = 0` for one-shot; any nonzero value for periodic.
- **Dynamic parameter updates** – modify rate, channel, power, interval, etc., at runtime.
- **Hardware and software retries** – configurable HW retry limit and SW retry fallback with lifetime retry counters.
- **Latency compensation** – exponential moving average of scheduler and channel-switch latency fed back into deadline calculation.
- **Power mapping** – custom callback to map dBm to hardware TX power percentage.
- **Channel switching** – per-injector channel selection.
- **Access category selection** – BE, BK, VI, VO.
- **Statistics** – per-injector and global packet/error counts; `injectorManager_resetStats()` to clear.
- **Thread-safe** – all public functions protected by a mutex; generation counter prevents ABA races.
- **SMP-safe timer** – spinlock-based critical section when `portMUX_TYPE` is available.

---

## API Reference

### Error Codes

| Code | Value | Description |
|------|-------|-------------|
| `INJ_OK` | 0 | Success |
| `INJ_ERR` | -1 | General error |
| `INJ_ERR_NOT_FOUND` | -2 | Injector not found |
| `INJ_ERR_INVALID_ARG` | -3 | Invalid argument |
| `INJ_ERR_NO_SPACE` | -4 | No free slot / allocation failed |
| `INJ_ERR_TIMER` | -5 | Timer not initialised |
| `INJ_ERR_CHANNEL` | -6 | Invalid channel value / channel set failed |
| `INJ_ERR_BUSY` | -7 | Resource busy (reserved) |
| `INJ_ERR_RATE` | -8 | Invalid rate value |
| `INJ_ERR_POWER` | -9 | Power setting failed (reserved) |
| `INJ_ERR_STATE` | -10 | Invalid state (e.g., scheduler already running) |

---

### Configuration Macros

| Macro | Default | Description |
|-------|---------|-------------|
| `INJECTOR_NAME_MAX` | 32 | Maximum injector name length (bytes) |
| `INJECTOR_MAX` | 16 | Maximum number of injectors |
| `INJECTOR_MAX_PACKET_SIZE` | 2048 | Maximum packet size (bytes) |
| `INJECTOR_STOP_TIMEOUT_MS` | 200 | ms to wait for clean scheduler exit before force-delete |
| `INJECTOR_SPIN_GUARD_NS` | 2 000 000 | ns guard window before deadline where scheduler yields instead of sleeping. Set to 0 to disable. |

Override `INJECTOR_STOP_TIMEOUT_MS` and `INJECTOR_SPIN_GUARD_NS` by defining them before including `inject.h`.

---

### Types

#### `inject_rate_t`

```c
typedef enum {
    INJ_RATE_1M  = 0x02,  INJ_RATE_2M   = 0x04,
    INJ_RATE_5_5M= 0x0B,  INJ_RATE_11M  = 0x16,
    INJ_RATE_6M  = 0x0C,  INJ_RATE_9M   = 0x12,
    INJ_RATE_12M = 0x18,  INJ_RATE_18M  = 0x24,
    INJ_RATE_24M = 0x30,  INJ_RATE_36M  = 0x48,
    INJ_RATE_48M = 0x60,  INJ_RATE_54M  = 0x6C,
    INJ_RATE_MCS0= 0x80,  INJ_RATE_MCS1 = 0x81,
    INJ_RATE_MCS2= 0x82,  INJ_RATE_MCS3 = 0x83,
    INJ_RATE_MCS4= 0x84,  INJ_RATE_MCS5 = 0x85,
    INJ_RATE_MCS6= 0x86,  INJ_RATE_MCS7 = 0x87,
    INJ_RATE_DEFAULT = INJ_RATE_1M
} inject_rate_t;
```

#### `inject_flags_t`

| Flag | Value | Description |
|------|-------|-------------|
| `INJ_FLAG_NONE` | 0 | No flags |
| `INJ_FLAG_NO_ACK` | 1<<0 | Disable acknowledgment |
| `INJ_FLAG_USE_SHORT_GI` | 1<<1 | Use short guard interval |
| `INJ_FLAG_AGGREGATE` | 1<<2 | Enable aggregation (if supported) |
| `INJ_FLAG_FIXED_CHANNEL` | 1<<3 | Reserved |

#### `inject_ac_t`

```c
typedef enum {
    INJ_AC_BE = 0,   // Best effort
    INJ_AC_BK = 1,   // Background
    INJ_AC_VI = 2,   // Video
    INJ_AC_VO = 3    // Voice
} inject_ac_t;
```

#### `InjectorInfo`

```c
typedef struct {
    char           name[INJECTOR_NAME_MAX];
    bool           active;
    uint8_t        channel;
    uint64_t       interval_ns;   // 0 = single-shot
    uint32_t       maxPackets;
    uint32_t       packets_sent;
    uint32_t       tx_errors;
    uint32_t       tx_retries;    // Lifetime SW retry total
    uint8_t        maxRetries;    // HW retry limit
    inject_rate_t  tx_rate;
    int8_t         tx_power_dbm;
    inject_flags_t flags;
    inject_ac_t    ac_queue;
    uint32_t       packetLen;
} InjectorInfo;
```

> **`tx_retries`** is the **lifetime total** of software retry attempts
> accumulated since the injector was created or last reset via
> `injectorManager_resetStats()`.

---

### Manager Lifecycle

#### `void injector_set_timer_freq_hz(uint32_t freq)`

Sets the frequency of the high-resolution hardware timer (Hz).

> ⚠️ **Must be called BEFORE `injectorManager_create()`.**
> Calling `create()` first will always return NULL.

#### `injectorManager* injectorManager_create(void)`

Creates a new manager instance.  Returns `NULL` if the timer frequency has not
been set, if memory allocation fails, or if semaphore creation fails.

#### `void injectorManager_destroy(injectorManager *mgr)`

Stops the scheduler task (with timeout + force-delete), frees all injectors and
releases the manager.

> **Warning:** Do not call from the scheduler task.
> After this call, the pointer passed by the caller is dangling — set it to
> `NULL` immediately.

#### `void injectorManager_clearAll(injectorManager *mgr)`

Removes all injectors.  Scheduler state is unaffected.

#### `void injectorManager_setWlanIndex(injectorManager *mgr, uint8_t wlan_idx)`

Sets the Wi-Fi interface index (default: `STA_WLAN_INDEX`).

---

### Injector Management

#### `int injectorManager_setInjectorEx(...)`

Creates or reconfigures a named injector (inactive by default).

**If an injector with the same name already exists**, it is reconfigured in
place and **all statistics and timing-compensation accumulators are reset**.
Activate it separately with `injectorManager_activateInjector()`.

```c
int injectorManager_setInjectorEx(
    injectorManager *mgr,
    const char      *name,           // unique name
    const uint8_t   *packetData,     // payload (copied internally)
    uint32_t         packetLen,      // ≤ INJECTOR_MAX_PACKET_SIZE
    uint8_t          channel,        // 0 = use current; 1-13; 36-165
    uint64_t         start_time_ns,  // absolute start time; 0 = now
    uint64_t         interval_ns,    // period (ns); 0 = single-shot
    uint32_t         maxPackets,     // 0 = unlimited; ignored if interval_ns==0
    uint8_t          hwRetries,      // hardware retry count
    uint8_t          swRetries,      // software retry attempts on HW failure
    inject_rate_t    rate,           // must be a member of inject_rate_t
    int8_t           tx_power_dbm,   // -1 = use current/default
    inject_flags_t   flags,
    inject_ac_t      ac_queue
);
```

Returns `INJ_ERR_CHANNEL` for an invalid channel, `INJ_ERR_RATE` for an
unrecognised rate value.

**`interval_ns = 0`** – single-shot: the injector transmits exactly one packet
then transitions to `DONE`.  `maxPackets` is ignored in this mode.

#### `int injectorManager_setInjector(...)` (inline convenience)

Wrapper with defaults: `start_time_ns=0`, `swRetries=5`, `rate=1M`,
`tx_power_dbm=-1`, `flags=NONE`, `ac_queue=BE`.

#### `int injectorManager_deleteInjector(mgr, name)`
#### `int injectorManager_activateInjector(mgr, name)`
#### `int injectorManager_deactivateInjector(mgr, name)`

---

### Dynamic Parameter Updates

All setters are thread-safe and generation-stamped to prevent ABA races when
called concurrently with the scheduler.

```
injectorManager_setRate(mgr, name, rate)         // INJ_ERR_RATE if unknown rate
injectorManager_setChannel(mgr, name, channel)   // INJ_ERR_CHANNEL if invalid channel
injectorManager_setTxPower(mgr, name, tx_power_dbm)
injectorManager_setIntervalNs(mgr, name, interval_ns)   // 0 = single-shot
injectorManager_setMaxPackets(mgr, name, maxPackets)
injectorManager_setHwRetries(mgr, name, hwRetries)
injectorManager_setSwRetries(mgr, name, swRetries)
injectorManager_setFlags(mgr, name, flags)
injectorManager_setAcQueue(mgr, name, ac_queue)
injectorManager_setPacketData(mgr, name, data, len)
```

---

### Statistics

#### `int injectorManager_getInfo(mgr, name, &info)`

Fills an `InjectorInfo` struct.  `info.tx_retries` is the lifetime SW-retry
total.

#### `int injectorManager_listInjectors(mgr, names, maxCount)`

Returns the number of names written.

#### `uint64_t injectorManager_getTotalPackets(mgr)`
#### `uint32_t injectorManager_getActiveCount(mgr)`
#### `uint32_t injectorManager_getTotalErrors(mgr)`

#### `int injectorManager_resetStats(mgr, name)`

Clears `packets_sent`, `tx_errors`, and `tx_retries` for the named injector.
Does not affect scheduling state or timing-compensation EMAs.

---

### Scheduler Control

#### `int injectorManager_startSchedulerTask(mgr, priority)`

Creates the scheduler task with a default stack of 2048 bytes.

#### `int injectorManager_startSchedulerTaskEx(mgr, priority, stackSize)`

Same, with an explicit stack size.  Recommended minimum is 2048 bytes; increase
if your `wifi_send_raw_frame()` implementation uses significant stack.

#### `int injectorManager_stopSchedulerTask(mgr)`

Signals the scheduler to stop, waits up to `INJECTOR_STOP_TIMEOUT_MS` ms for a
clean exit, then force-deletes the task if it has not exited.

- Returns `INJ_ERR_STATE` if no scheduler is running.
- Returns `INJ_ERR_STATE` (and fires `configASSERT`) if called from the
  scheduler task itself.

> **Force-delete warning:** if the scheduler is blocked inside
> `wifi_send_raw_frame()` when the timeout fires, the task is deleted in a
> potentially unsafe state.  Increase `INJECTOR_STOP_TIMEOUT_MS` to a value
> larger than the worst-case `wifi_send_raw_frame()` latency on your platform.

---

### Power Mapping

#### `void injectorManager_setPowerMappingCallback(mgr, callback)`

Registers a `uint8_t (*callback)(int8_t dbm)` to convert a dBm value to a
hardware percentage.  Default mapping:

| dBm | % |
|-----|---|
| ≤ 0 | 13 |
| ≤ 5 | 25 |
| ≤ 10 | 50 |
| ≤ 15 | 75 |
| > 15 | 100 |

---

## Platform Requirements

The following symbols must be provided by the application or BSP:

| Symbol | Signature | Description |
|--------|-----------|-------------|
| `init_highres_timer` | `void (void)` | Initialise the hardware timer |
| `read_highres_timer` | `uint32_t (void)` | Read raw 32-bit timer counter |
| `wifi_set_channel` | `int (uint8_t wlan_idx, uint8_t ch)` | Change Wi-Fi channel |
| `wifi_send_raw_frame` | `int (struct rtw_raw_frame_desc*)` | Transmit raw 802.11 frame |
| `wifi_set_tx_power_percentage` | `int (uint8_t wlan_idx, uint8_t pct)` | Set TX power (weak default = no-op) |

`platform_get_time_ns()` is implemented internally using `init_highres_timer` /
`read_highres_timer` and the frequency set by `injector_set_timer_freq_hz()`.

---

## Example Usage

```c
#include "inject.h"

static injectorManager *g_mgr = NULL;

// Custom TX power map (linear 0–20 dBm → 0–100%)
uint8_t my_power_map(int8_t dbm) {
    if (dbm <= 0)  return 10;
    if (dbm >= 20) return 100;
    return (uint8_t)((dbm * 100) / 20);
}

#define TIMER_FREQ_HZ  80000000UL   // 80 MHz hardware timer

void app_main(void) {
    // ── Step 1: set timer frequency FIRST ─────────────────────────────
    injector_set_timer_freq_hz(TIMER_FREQ_HZ);

    // ── Step 2: create manager ─────────────────────────────────────────
    g_mgr = injectorManager_create();
    if (!g_mgr) { /* handle error */ return; }

    // ── Step 3: configure interface and callbacks ──────────────────────
    injectorManager_setWlanIndex(g_mgr, 0);
    injectorManager_setPowerMappingCallback(g_mgr, my_power_map);

    // ── Step 4: define a periodic beacon injector ──────────────────────
    uint8_t beacon[] = { /* 802.11 frame bytes */ };

    int ret = injectorManager_setInjector(
        g_mgr,
        "beacon",          // name
        beacon,            // payload
        sizeof(beacon),    // length
        6,                 // channel 6
        102400000ULL,      // 102.4 ms interval (~9.8 Hz)
        0,                 // unlimited packets
        3                  // 3 HW retries
    );
    if (ret != INJ_OK) { /* handle error */ }

    // ── Step 5: define a single-shot probe request ─────────────────────
    uint8_t probe[] = { /* 802.11 probe request */ };

    ret = injectorManager_setInjectorEx(
        g_mgr, "probe_once",
        probe, sizeof(probe),
        1,                 // channel 1
        0,                 // start now
        0,                 // interval_ns = 0  → single-shot
        0,                 // maxPackets ignored for single-shot
        2,                 // hwRetries
        3,                 // swRetries
        INJ_RATE_6M,
        10,                // 10 dBm
        INJ_FLAG_NO_ACK,
        INJ_AC_BE
    );
    if (ret != INJ_OK) { /* handle error */ }

    // ── Step 6: start scheduler task (priority 5, default stack) ──────
    ret = injectorManager_startSchedulerTask(g_mgr, 5);
    if (ret != INJ_OK) { /* handle error */ }

    // ── Step 7: activate injectors ────────────────────────────────────
    injectorManager_activateInjector(g_mgr, "beacon");
    injectorManager_activateInjector(g_mgr, "probe_once");

    // ── Later: read statistics ─────────────────────────────────────────
    InjectorInfo info;
    if (injectorManager_getInfo(g_mgr, "beacon", &info) == INJ_OK) {
        printf("beacon: sent=%u errors=%u retries=%u\n",
               info.packets_sent, info.tx_errors, info.tx_retries);
    }

    // ── Shutdown ───────────────────────────────────────────────────────
    injectorManager_stopSchedulerTask(g_mgr);
    injectorManager_destroy(g_mgr);
    g_mgr = NULL;   // clear dangling pointer
}
```

---

## Notes and Limitations

### Timing accuracy

The scheduler uses a two-phase sleep strategy:
1. **Tick sleep** for `(deadline − INJECTOR_SPIN_GUARD_NS)` using
   `xSemaphoreTake()` with a tick timeout.  An early wake via `activateInjector`
   or `stopSchedulerTask` causes immediate re-evaluation.
2. **Yield spin** for the remaining `INJECTOR_SPIN_GUARD_NS` window, calling
   `taskYIELD()` each iteration.

At `INJECTOR_SPIN_GUARD_NS = 2 ms` and a 100 Hz tick rate this gives ~2 ms
accuracy at the cost of one `taskYIELD()` per 2 ms per active injector.  Set
the guard to 0 for pure tick-based sleeping.

### Latency compensation

After each successful transmission the scheduler adjusts the next deadline by
subtracting an exponential moving average of the scheduler latency and the
channel-switch+TX time.  Compensation is capped at half the interval to
prevent runaway drift.

### Channel switching

Switching channels on every packet adds measurable latency (typically 1–5 ms).
Group injectors by channel or assign the same channel to all injectors to avoid
inter-packet channel thrashing.

### Memory

Each injector allocates a heap buffer for its packet payload.  Under heavy load
the per-send `pvPortMalloc` / `vPortFree` pair is performed **outside** the
manager mutex to avoid blocking API callers.

### SMP (multi-core)

`platform_get_time_ns()` uses a `portMUX_TYPE` spinlock when `portMUX_TYPE` is
defined (FreeRTOS-SMP, ESP-IDF).  Single-core builds fall back to
`taskENTER_CRITICAL()`.

### Stopping safely

`injectorManager_stopSchedulerTask()` and `injectorManager_destroy()` both wait
up to `INJECTOR_STOP_TIMEOUT_MS` ms for a clean exit.  If the task is blocked
inside `wifi_send_raw_frame()` past this deadline it is force-deleted, which
may leave the Wi-Fi driver in an undefined state.  Choose
`INJECTOR_STOP_TIMEOUT_MS` conservatively for your hardware.

---

## License

This project is licensed under the GPLv2.
