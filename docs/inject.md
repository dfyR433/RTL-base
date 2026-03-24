# Wi‑Fi Packet Injector Manager

## Introduction

The **Wi‑Fi Packet Injector Manager** is a real‑time library for high‑precision, scheduled transmission of raw Wi‑Fi frames on embedded systems running FreeRTOS. It allows creating multiple named injectors, each with its own packet payload, transmission rate, channel, power, retry policies, and scheduling parameters. The library uses a nanosecond‑resolution timer and a dedicated scheduler task to achieve accurate packet timing.

## Features

- **Multiple independent injectors** – up to 16 named injectors (configurable).
- **High‑precision scheduling** – nanosecond‑accurate transmission times.
- **Flexible timing** – absolute start time, periodic intervals, or single‑shot.
- **Dynamic parameter updates** – modify rate, channel, power, interval, etc., at runtime.
- **Hardware and software retries** – configurable HW retry limit and SW retry fallback.
- **Power mapping** – custom callback to map dBm to hardware‑specific TX power percentage.
- **Channel switching** – per‑injector channel selection (if different from current).
- **Various transmit rates** – supports 802.11b/g/n rates (1Mbps to MCS7).
- **Access category selection** – BE, BK, VI, VO.
- **Flags** – disable ACK, short GI, aggregation, fixed channel.
- **Statistics** – per‑injector and global packet/error counts.
- **Thread‑safe** – all public functions are protected by a mutex.
- **Lightweight** – uses only FreeRTOS and platform‑specific Wi‑Fi APIs.

## API Reference

### Error Codes

| Code                | Value | Description                         |
|---------------------|-------|-------------------------------------|
| `INJ_OK`            | 0     | Success                             |
| `INJ_ERR`           | -1    | General error                       |
| `INJ_ERR_NOT_FOUND` | -2    | Injector not found                  |
| `INJ_ERR_INVALID_ARG` | -3 | Invalid argument                    |
| `INJ_ERR_NO_SPACE`  | -4    | No free slot / memory allocation failed |
| `INJ_ERR_TIMER`     | -5    | Timer not initialized               |
| `INJ_ERR_CHANNEL`   | -6    | Channel setting failed              |
| `INJ_ERR_BUSY`      | -7    | Resource busy (reserved)            |
| `INJ_ERR_RATE`      | -8    | Invalid rate (reserved)             |
| `INJ_ERR_POWER`     | -9    | Power setting failed (reserved)     |
| `INJ_ERR_STATE`     | -10   | Invalid state (e.g., scheduler already running) |

### Configuration Macros

| Macro                      | Default | Description                         |
|----------------------------|---------|-------------------------------------|
| `INJECTOR_NAME_MAX`        | 32      | Maximum length of injector name     |
| `INJECTOR_MAX`             | 16      | Maximum number of injectors         |
| `INJECTOR_MAX_PACKET_SIZE` | 2048    | Maximum packet size (bytes)         |

### Types

#### `inject_rate_t`

```c
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
```

#### `inject_flags_t`

| Flag                   | Value  | Description                         |
|------------------------|--------|-------------------------------------|
| `INJ_FLAG_NONE`        | 0      | No flags                            |
| `INJ_FLAG_NO_ACK`      | 1<<0   | Disable acknowledgment              |
| `INJ_FLAG_USE_SHORT_GI`| 1<<1   | Use short guard interval            |
| `INJ_FLAG_AGGREGATE`   | 1<<2   | Enable aggregation (if supported)   |
| `INJ_FLAG_FIXED_CHANNEL`| 1<<3  | (Reserved)                          |

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

Structure to retrieve injector status.

```c
typedef struct {
    char name[INJECTOR_NAME_MAX];
    bool active;
    uint8_t channel;
    uint64_t interval_ns;
    uint32_t maxPackets;
    uint32_t packets_sent;
    uint32_t tx_errors;
    uint32_t tx_retries;      // software retry attempts
    uint8_t  maxRetries;      // hardware retry limit
    inject_rate_t tx_rate;
    int8_t   tx_power_dbm;
    inject_flags_t flags;
    inject_ac_t ac_queue;
    uint32_t packetLen;
} InjectorInfo;
```

---

### Manager Lifecycle

#### `injectorManager* injectorManager_create(void)`

Creates a new injector manager instance. Allocates memory, initialises mutexes, and prepares the internal timer.

- **Returns:** Pointer to manager, or `NULL` on failure.

#### `void injectorManager_destroy(injectorManager *mgr)`

Destroys a manager instance. Stops the scheduler task (if running), frees all injector resources, and releases memory.

- **Parameters:** `mgr` – manager instance.

#### `void injectorManager_clearAll(injectorManager *mgr)`

Removes all injectors from the manager. Does not affect the scheduler state.

- **Parameters:** `mgr` – manager instance.

#### `void injectorManager_setWlanIndex(injectorManager *mgr, uint8_t wlan_idx)`

Sets the Wi‑Fi interface index used for all transmissions. The default is `STA_WLAN_INDEX` (usually 0).

- **Parameters:**
  - `mgr` – manager instance.
  - `wlan_idx` – interface index (e.g., 0 for station).

#### `void injector_set_timer_freq_hz(uint32_t freq)`

**Must be called before any injector creation** to set the frequency of the high‑resolution timer (in Hz). This value is used to convert timer counts to nanoseconds.

- **Parameters:** `freq` – timer frequency in Hz.

#### `void injectorManager_setLogCallback(inj_log_cb_t cb)`

Sets a callback for logging messages. If no callback is set, `printf` is used.

- **Parameters:** `cb` – function pointer with signature `void (*cb)(const char *msg)`.

---

### Injector Management

#### `int injectorManager_setInjectorEx(...)`

Creates or updates an injector (inactive by default) with full parameter control. The packet data is copied internally, so the caller may free its buffer after the call.

```c
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
```

- **Parameters:**
  - `mgr` – manager instance.
  - `name` – unique name for the injector (max `INJECTOR_NAME_MAX-1` chars).
  - `packetData` – raw frame data (must include 802.11 header and FCS if not added by hardware).
  - `packetLen` – length of packet (max `INJECTOR_MAX_PACKET_SIZE`).
  - `channel` – Wi‑Fi channel (1-13, 36-165). 0 means "use current".
  - `start_time_ns` – absolute nanosecond timestamp when the injector should start. If `0`, start immediately.
  - `interval_ns` – interval between successive transmissions (nanoseconds). 0 means single‑shot (only one packet).
  - `maxPackets` – maximum number of packets to send; 0 = unlimited.
  - `hwRetries` – hardware retry count (passed to the Wi‑Fi driver).
  - `swRetries` – software retry attempts if hardware fails.
  - `rate` – transmit rate (see `inject_rate_t`).
  - `tx_power_dbm` – transmit power in dBm; -1 = use default.
  - `flags` – bitwise OR of `inject_flags_t`.
  - `ac_queue` – access category.
- **Returns:** `INJ_OK` on success, error code otherwise.

#### `static inline int injectorManager_setInjector(...)`

Simplified version with default values: `start_time_ns=0`, `swRetries=5`, `rate=INJ_RATE_DEFAULT`, `tx_power_dbm=-1`, `flags=INJ_FLAG_NONE`, `ac_queue=INJ_AC_BE`.

```c
int injectorManager_setInjector(injectorManager *mgr,
                                const char *name,
                                const uint8_t *packetData,
                                uint32_t packetLen,
                                uint8_t channel,
                                uint64_t interval_ns,
                                uint32_t maxPackets,
                                uint8_t hwRetries);
```

#### `int injectorManager_deleteInjector(injectorManager *mgr, const char *name)`

Deletes an injector. If the injector is active, it is deactivated first.

- **Returns:** `INJ_OK` or `INJ_ERR_NOT_FOUND`.

#### `int injectorManager_activateInjector(injectorManager *mgr, const char *name)`

Activates an existing injector. The scheduler will begin transmitting according to its parameters.

- **Returns:** `INJ_OK` or `INJ_ERR_NOT_FOUND`.

#### `int injectorManager_deactivateInjector(injectorManager *mgr, const char *name)`

Deactivates an injector. No further packets will be sent, but the injector remains configured.

- **Returns:** `INJ_OK` or `INJ_ERR_NOT_FOUND`.

---

### Dynamic Parameter Updates

All these functions modify a single parameter of an existing injector. The change takes effect immediately (next transmission uses the new value).

- `int injectorManager_setRate(...)`
- `int injectorManager_setChannel(...)`
- `int injectorManager_setTxPower(...)`
- `int injectorManager_setIntervalNs(...)`
- `int injectorManager_setMaxPackets(...)`
- `int injectorManager_setHwRetries(...)`
- `int injectorManager_setSwRetries(...)`
- `int injectorManager_setFlags(...)`
- `int injectorManager_setAcQueue(...)`

**Signature example:**
```c
int injectorManager_setRate(injectorManager *mgr, const char *name, inject_rate_t rate);
```
All return `INJ_OK` or `INJ_ERR_NOT_FOUND`.

#### `int injectorManager_setPacketData(injectorManager *mgr, const char *name, const uint8_t *newData, uint32_t newLen)`

Replaces the packet payload of an injector. The new data is copied internally.

- **Returns:** `INJ_OK`, `INJ_ERR_NOT_FOUND`, or `INJ_ERR_NO_SPACE` (memory allocation failure).

---

### Information and Statistics

#### `int injectorManager_getInfo(injectorManager *mgr, const char *name, InjectorInfo *info)`

Fills an `InjectorInfo` structure with the current state of an injector.

- **Returns:** `INJ_OK` or `INJ_ERR_NOT_FOUND`.

#### `int injectorManager_listInjectors(injectorManager *mgr, char names[][INJECTOR_NAME_MAX], int maxCount)`

Fills an array of strings with the names of all defined injectors. Returns the number of names copied.

- **Parameters:**
  - `names` – array of strings (each of length `INJECTOR_NAME_MAX`).
  - `maxCount` – size of the `names` array.
- **Returns:** number of names written (could be less than `maxCount`).

#### `uint64_t injectorManager_getTotalPackets(injectorManager *mgr)`

Returns the total number of packets successfully transmitted by all injectors.

#### `uint32_t injectorManager_getActiveCount(injectorManager *mgr)`

Returns the number of currently active injectors.

#### `uint32_t injectorManager_getTotalErrors(injectorManager *mgr)`

Returns the total number of transmission errors (after exhausting software retries) across all injectors.

---

### Scheduler Control

#### `int injectorManager_startSchedulerTask(injectorManager *mgr, UBaseType_t priority)`

Creates the scheduler task with a default stack size (2048 bytes). The task runs a tight loop checking which injectors are due for transmission.

- **Parameters:**
  - `mgr` – manager instance.
  - `priority` – FreeRTOS task priority.
- **Returns:** `INJ_OK` or `INJ_ERR` (if task creation fails) or `INJ_ERR_STATE` (if already running).

#### `int injectorManager_startSchedulerTaskEx(injectorManager *mgr, UBaseType_t priority, uint32_t stackSize)`

Same as above, but allows specifying the task stack size.

#### `int injectorManager_stopSchedulerTask(injectorManager *mgr)`

Stops the scheduler task. Waits up to 100 ms for the task to exit. If the task does not terminate gracefully, it is forcibly deleted.

- **Returns:** `INJ_OK` or `INJ_ERR_STATE` (if no task running).

---

### Power Mapping

#### `void injectorManager_setPowerMappingCallback(injectorManager *mgr, uint8_t (*callback)(int8_t dbm))`

Sets a callback to convert a requested dBm value to a hardware‑specific percentage (0‑100). The callback is invoked every time the TX power is changed. The default mapping is:

| dBm   | Percentage |
|-------|-----------|
| ≤0    | 13        |
| ≤5    | 25        |
| ≤10   | 50        |
| ≤15   | 75        |
| >15   | 100       |

- **Parameters:**
  - `mgr` – manager instance.
  - `callback` – function that takes dBm and returns percentage (0‑100).

---

## Platform Requirements

The library expects the following platform‑specific functions to be implemented by the application (or linked from the hardware abstraction layer):

### `uint64_t platform_get_time_ns(void)`

Returns the current time in nanoseconds since an arbitrary epoch. Must be monotonic and high‑resolution.

### `void injector_scheduler_wait_until_ns(uint64_t target_ns)`

Blocks the calling task (the scheduler) until the system time reaches `target_ns`. This is used to achieve accurate timing. The implementation typically uses a hardware timer or a busy‑wait with `taskYIELD()`.

### `void init_highres_timer(void)`

Initialises the high‑resolution timer used for time measurement. Called automatically by `platform_get_time_ns()` on first use.

### `uint32_t read_highres_timer(void)`

Returns the current count of the high‑resolution timer.

### `int wifi_set_tx_power_percentage(uint8_t wlan_idx, uint8_t percentage)`

Sets the TX power of the Wi‑Fi interface to the given percentage of maximum. This function is weakly defined to allow application‑specific implementation. The default (weak) version returns 0 (success). **The application must provide a real implementation.**

### `int wifi_set_channel(uint8_t wlan_idx, uint8_t channel)`

Sets the Wi‑Fi channel of the interface. Must be provided by the application.

### `int wifi_send_raw_frame(struct rtw_raw_frame_desc *desc)`

Sends a raw 802.11 frame using the provided descriptor. The descriptor structure is defined by the Wi‑Fi driver (here `rtw_raw_frame_desc`). The library fills the following fields:

- `wlan_idx`
- `buf` / `buf_len`
- `tx_rate`
- `retry_limit`
- `ac_queue`
- `sgi`
- `agg_en`

**Note:** The application is responsible for including the FCS (Frame Check Sequence) in the packet data if required by the hardware.

---

## Example Usage

```c
#include "inject.h"

// Global manager pointer
static injectorManager *g_mgr = NULL;

// Callback for TX power mapping (example: linear mapping)
uint8_t my_power_map(int8_t dbm) {
    if (dbm <= 0) return 10;
    if (dbm >= 20) return 100;
    return (uint8_t)((dbm * 100) / 20);
}

// Timer frequency (e.g., 80 MHz)
#define TIMER_FREQ_HZ 80000000

void main(void) {
    // 1. Create manager
    g_mgr = injectorManager_create();
    if (!g_mgr) {
        // handle error
    }

    // 2. Set timer frequency (must be done before any injector)
    injector_set_timer_freq_hz(TIMER_FREQ_HZ);

    // 3. Set WLAN interface index (usually 0)
    injectorManager_setWlanIndex(g_mgr, 0);

    // 4. Set power mapping callback (optional)
    injectorManager_setPowerMappingCallback(g_mgr, my_power_map);

    // 5. Set logging callback (optional)
    injectorManager_setLogCallback(my_log_callback);

    // 6. Define packet payload (e.g., a beacon frame)
    uint8_t beacon[] = { ... }; // 802.11 frame with header and FCS

    // 7. Create an injector (will be inactive)
    int ret = injectorManager_setInjector(g_mgr,
                                          "beacon_inj",
                                          beacon,
                                          sizeof(beacon),
                                          6,               // channel 6
                                          1000000000ULL,   // interval = 1 second
                                          0,               // unlimited packets
                                          3);              // 3 hardware retries
    if (ret != INJ_OK) {
        // handle error
    }

    // 8. Start the scheduler task
    ret = injectorManager_startSchedulerTask(g_mgr, 5); // priority 5
    if (ret != INJ_OK) {
        // handle error
    }

    // 9. Activate the injector
    ret = injectorManager_activateInjector(g_mgr, "beacon_inj");
    if (ret != INJ_OK) {
        // handle error
    }

    // ... application continues ...

    // Later: stop and clean up
    injectorManager_stopSchedulerTask(g_mgr);
    injectorManager_destroy(g_mgr);
}
```

---

## Notes and Limitations

- **Timer initialisation:** `injector_set_timer_freq_hz()` **must** be called before creating any injector. The timer frequency must match the hardware timer used by `read_highres_timer()`.
- **Packet data:** The library copies the packet data internally, so the caller may free its buffer after `setInjector`. The data must include the full 802.11 MAC header and FCS unless the hardware adds it automatically.
- **Channel switching:** The scheduler changes the channel only if an injector specifies a non‑zero channel different from the current channel. If multiple injectors use different channels, the channel will change on every transmission, which may cause delays. It is advisable to group injectors by channel or use `INJ_FLAG_FIXED_CHANNEL` (reserved) to prevent changes.
- **Power setting:** The `wifi_set_tx_power_percentage()` function is weakly defined and must be provided by the application if TX power control is needed.
- **Real‑time behaviour:** The scheduler task uses a busy‑wait (or `injector_scheduler_wait_until_ns`) to achieve nanosecond accuracy. This may consume CPU time; adjust task priority and stack size accordingly.
- **Concurrency:** All public functions are thread‑safe, but the scheduler task should not be modified directly.
- **Memory:** Each injector allocates its own packet buffer. The maximum number of injectors is limited by `INJECTOR_MAX`.

---

## License

This project is licensed under the GPLv2
