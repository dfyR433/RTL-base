# Monitor Library — `monitor.h` / `monitor.c`

**Target:** RTL8721Dx (Ameba) — KM4 core (ARM Cortex-M55)  
**Purpose:** Wi-Fi monitor-mode packet capture with pcapng/pcap/raw output, adaptive channel hopping, and lock-free zero-copy processing  
**Thread safety:** Public API functions are safe to call from any task. The promisc callback path is ISR-safe (no OS calls, atomic operations only).

---

## Overview

The monitor library puts the RTL8721Dx radio into promiscuous (monitor) mode and captures raw 802.11 frames with nanosecond timestamps. The processing pipeline is split across three FreeRTOS tasks and a promisc callback to separate timing-critical capture from I/O-bound output.

### Pipeline

```
Radio hardware
      │
      ▼ (promisc callback — ISR context, atomic only)
 Lock-free pool allocator  ←─ 64-slot bitmask CAS allocator
      │
      ▼
 Lock-free SPSC ring buffer  (1024 slots)
      │
      ▼ monitor_task  (prio +3)
  Frame filter & dispatch
  ├── MAC filter (linear-probe hash table)
  ├── BPF rules  (offset/mask/value matcher)
  ├── Frame-type filter
  ├── RSSI threshold filter
  └── Radiotap header construction
            │
            ▼ writer_task queue  (128 slots)
       Block pool allocator  ←─ 32-slot fixed-size blocks
            │
            ▼ writer_task  (prio +2)
       Output format serialization
       ├── pcapng (SHB + IDB + EPB, nanosecond timestamps)  [default]
       ├── pcap   (global header + packet records, µs timestamps)
       └── raw    (bare 802.11 frame, no header)
            │
            ▼
       monitor_sink_cb_t  (user-supplied: UART, USB CDC, VFS, ...)

hop_task  (prio +1) ← adaptive channel dwell (50–300 ms)
```

### Memory model

All hot-path allocations use dedicated pool allocators — no heap calls in the capture path. The promisc callback uses a **lock-free bitmask CAS allocator** (64 slots × 2346 bytes = ~143 KB). The writer task uses a **fixed-size block pool** (32 slots × 2448 bytes = ~76 KB). These are statically allocated and must fit in KM4 SRAM.

---

## Compile-time Output Format

Select the output format at build time. Default is pcapng.

```makefile
-DMONITOR_FORMAT=0   # MONITOR_FMT_CAP   — raw 802.11 frames, no header
-DMONITOR_FORMAT=1   # MONITOR_FMT_PCAP  — libpcap, µs resolution
-DMONITOR_FORMAT=2   # MONITOR_FMT_PCAPNG — pcapng, ns resolution (default)
```

Attempting to define `MONITOR_FORMAT` to any other value causes a compile-time `#error`.

---

## Configuration Macros

### Channel Hopping

| Macro | Default | Description |
|-------|---------|-------------|
| `CHANNEL_LIST_LEN` | `38` | Number of channels in the hop list (2.4 GHz 1–14 + 5 GHz channels) |
| `HOP_DWELL_MIN_MS` | `50` | Minimum dwell time per channel in adaptive mode |
| `HOP_DWELL_MAX_MS` | `300` | Maximum dwell time per channel in adaptive mode |
| `HOP_TRAFFIC_HIGH_THR` | `20` | Frames-per-dwell threshold above which dwell time is extended toward `HOP_DWELL_MAX_MS` |

### Task Parameters

| Macro | Default | Description |
|-------|---------|-------------|
| `MONITOR_TASK_STACK` | `4096` | Monitor task stack in bytes |
| `HOP_TASK_STACK` | `1024` | Channel hop task stack in bytes |
| `WRITER_TASK_STACK` | `2048` | Writer task stack in bytes |
| `MONITOR_TASK_PRIO` | `tskIDLE + 3` | Monitor task priority |
| `HOP_TASK_PRIO` | `tskIDLE + 1` | Hop task priority |
| `WRITER_TASK_PRIO` | `tskIDLE + 2` | Writer task priority |
| `MONITOR_JOIN_TIMEOUT_MS` | `2000` | `monitor_stop()` wait for monitor task exit |
| `HOP_JOIN_TIMEOUT_MS` | `500` | Wait for hop task exit |
| `WRITER_JOIN_TIMEOUT_MS` | `500` | Wait for writer task exit |

### Pool and Ring Sizing

| Macro | Default | Description |
|-------|---------|-------------|
| `PACKET_POOL_SIZE` | `64` | Frame pool slot count. Each slot holds one captured frame. |
| `PACKET_BUFFER_SIZE` | `2346` | Frame pool slot size in bytes (max 802.11 MPDU + FCS) |
| `RING_SIZE` | `1024` | SPSC ring buffer capacity (pointer-sized entries) |
| `BLOCK_POOL_SIZE` | `32` | Writer block pool slot count |
| `BLOCK_MAX_SIZE` | `2448` | Writer block slot size (frame + radiotap + pcapng EPB header overhead) |
| `WRITER_QUEUE_LEN` | `128` | FreeRTOS queue depth from monitor_task to writer_task |

### Filter Limits

| Macro | Default | Description |
|-------|---------|-------------|
| `MAX_MAC_ENTRIES` | `16` | Maximum MAC addresses in the filter table |
| `MAX_BPF_RULES` | `8` | Maximum BPF match rules |
| `WLAN_IDX` | `STA_WLAN_INDEX` | WiFi interface index for promisc registration |

---

## Types

### `monitor_bpf_rule_t` — Byte-level frame filter

```c
typedef struct {
    uint16_t offset;  // Byte offset from start of 802.11 header (after radiotap)
    uint8_t  mask;    // Bit mask applied before comparison
    uint8_t  value;   // Expected value: (frame[offset] & mask) == value
} monitor_bpf_rule_t;
```

Rules are ANDed together — a frame passes only if all rules match. Up to `MAX_BPF_RULES` rules may be active simultaneously. Evaluation occurs in `monitor_task`, before the writer queue.

**Example — match only QoS Data frames:**
```c
monitor_bpf_rule_t rules[] = {
    { .offset = 0, .mask = 0xFC, .value = 0x88 }, // FC byte 0: type=data(0x08), subtype QoS(0x80)
    { .offset = 1, .mask = 0x03, .value = 0x00 }, // FC byte 1: ToDS=0, FromDS=0 (IBSS)
};
monitor_set_bpf_rules(rules, 2);
```

---

### `monitor_filter_t` — Frame type filter

| Value | Passes |
|-------|--------|
| `FILTER_ALL` | All frame types |
| `FILTER_DATA` | Data frames only |
| `FILTER_MANAGEMENT` | Management frames only |
| `FILTER_CONTROL` | Control frames only |
| `FILTER_BEACON` | Beacon frames only |
| `FILTER_PROBE_REQ` | Probe Request frames only |
| `FILTER_PROBE_RSP` | Probe Response frames only |

Applied before BPF rules. Implemented by inspecting the Frame Control type/subtype fields.

---

### `mac_filter_mode_t` — MAC address filter mode

| Value | Behaviour |
|-------|-----------|
| `MAC_FILTER_NONE` | MAC filter disabled; all source addresses pass |
| `MAC_FILTER_ALLOWLIST` | Only frames whose source MAC is in the filter table pass |
| `MAC_FILTER_DENYLIST` | Frames whose source MAC is in the filter table are dropped |

The filter table is a linear-probing hash table with `MAX_MAC_ENTRIES` (16) slots. Lookup is O(1) average, O(n) worst-case. The hash key is the lower 4 bytes of the MAC address XOR'd with the upper 2 bytes.

---

### `channel_stats_t` — Per-channel statistics

Populated by `monitor_get_channel_stats()`. All fields are updated without locking (relaxed atomics); read coherence is best-effort.

| Field | Type | Description |
|-------|------|-------------|
| `frames_total` | `uint32_t` | Total frames captured on this channel |
| `frames_mgmt` | `uint32_t` | Management frame count |
| `frames_ctrl` | `uint32_t` | Control frame count |
| `frames_data` | `uint32_t` | Data frame count |
| `frames_other` | `uint32_t` | Frames with unknown type |
| `rssi_min` | `int8_t` | Minimum observed RSSI (dBm) |
| `rssi_max` | `int8_t` | Maximum observed RSSI (dBm) |
| `rssi_sum` | `int32_t` | Running sum for mean computation: `mean = rssi_sum / frames_total` |
| `rssi_m2` | `int64_t` | Welford M2 accumulator for online variance: `var = rssi_m2 / (frames_total - 1)` |
| `rssi_histogram[8]` | `uint32_t[]` | RSSI distribution across 8 bins (bin boundaries defined internally) |
| `traffic_ema` | `uint32_t` | Exponential moving average of frames per dwell period. Used by the adaptive hopper to decide dwell time. |

---

### `monitor_sink_cb_t` — Output sink

```c
typedef void (*monitor_sink_cb_t)(const void *data, size_t len);
```

Called from `writer_task` for each formatted output block. May block (UART DMA wait, VFS write, etc.). The writer task serializes all calls so the callback does not need to be re-entrant.

Typical implementations:
- **UART:** `serial_send_blocking(data, len)`
- **USB CDC:** `usb_cdc_write(data, len)`
- **VFS (SD card):** `f_write(&fil, data, len, &bw)`
- **Ring buffer to another task:** enqueue `(data, len)` into a second FreeRTOS queue

---

### `monitor_capture_cb_t` — Per-frame capture callback

```c
typedef void (*monitor_capture_cb_t)(const uint8_t *frame, uint32_t len,
                                      uint8_t channel, int8_t rssi,
                                      uint64_t ts_ns);
```

Called from `monitor_task` for every frame that passes all filters (type, RSSI threshold, MAC, BPF). `frame` points into the pool buffer — **do not store the pointer**; copy any needed data before returning. `ts_ns` is the `timer_get_time_ns_isr()` value captured in the promisc callback.

This callback runs concurrently with the writer pipeline. If you need to write to the same sink, you must synchronize externally.

---

### Global Statistics

Exported as `volatile uint32_t` for lock-free read from any context:

| Symbol | Description |
|--------|-------------|
| `stats_captured` | Total frames written to the SPSC ring |
| `stats_dropped_ring` | Frames dropped because the ring was full |
| `stats_dropped_pool` | Frames dropped because the packet pool was exhausted |
| `stats_dropped_block` | Frames dropped because the writer block pool was exhausted |
| `stats_peak_pool_used` | High-water mark of simultaneous pool slots in use |

---

## API Reference

### Lifecycle

#### `void monitor_start(void)`

Registers the promisc callback with the Realtek WiFi driver (`wifi_set_promisc()`), creates the monitor, hop, and writer FreeRTOS tasks, and writes the pcapng/pcap file header to the sink (if format is not raw). Safe to call once at startup; calling again without `monitor_stop()` is a no-op.

Before calling `monitor_start()`:
1. Set the output sink: `monitor_set_output_sink(cb)`
2. Optionally configure the channel: `monitor_set_fixed_channel(ch)` or `monitor_set_hopping()`
3. Optionally configure filters, callbacks, and BPF rules

---

#### `void monitor_stop(void)`

Signals all three tasks to exit and waits up to their respective join timeouts (`MONITOR_JOIN_TIMEOUT_MS`, `HOP_JOIN_TIMEOUT_MS`, `WRITER_JOIN_TIMEOUT_MS`). Flushes any pending writer queue entries. Deregisters the promisc callback. Safe to call from any task.

---

### Channel Control

#### `void monitor_set_fixed_channel(uint8_t ch)`

Locks the radio to channel `ch`. Disables the hop task. If the hop task is already running, it exits on its next iteration.

---

#### `void monitor_set_hopping(void)`

Enables adaptive channel hopping. The hop task cycles through the internal channel list, dwelling on each channel for between `HOP_DWELL_MIN_MS` and `HOP_DWELL_MAX_MS` milliseconds. Dwell time for a channel is extended proportionally when `traffic_ema > HOP_TRAFFIC_HIGH_THR`.

---

#### `uint8_t monitor_get_channel(void)`

Returns the currently active radio channel. Thread-safe.

---

### Output

#### `void monitor_set_output_sink(monitor_sink_cb_t cb)`

Registers the output sink callback. Must be called before `monitor_start()`. Replaces any previously registered sink.

---

### Frame Filtering

Filters are applied in this order: frame type → RSSI threshold → MAC filter → BPF rules. A frame must pass all active filters to be forwarded to the writer pipeline and the capture callback.

#### `void monitor_set_filter(monitor_filter_t filter)`

Sets the frame type filter. Default is `FILTER_ALL`. Takes effect immediately on the next received frame.

---

#### `void monitor_set_rssi_threshold(int8_t min_rssi)`

Drops frames with RSSI below `min_rssi` dBm. Default is `INT8_MIN` (no threshold). Useful for rejecting weak/noisy frames on busy channels.

---

#### `void monitor_set_mac_filter_mode(mac_filter_mode_t mode)`

Sets the MAC filter mode. Default is `MAC_FILTER_NONE`.

---

#### `int monitor_add_mac_filter(const uint8_t *mac)`

Adds a 6-byte MAC address to the filter table. Returns `0` on success, `-1` if the table is full (`MAX_MAC_ENTRIES` entries). The filter applies to the frame's source address field (bytes 10–15 of a standard 802.11 frame).

---

#### `int monitor_remove_mac_filter(const uint8_t *mac)`

Removes a MAC address from the filter table. Returns `0` on success, `-1` if the address was not found. Uses linear scan of the hash table.

---

#### `void monitor_clear_mac_filter(void)`

Removes all entries from the MAC filter table and resets mode to `MAC_FILTER_NONE`.

---

#### `void monitor_set_bpf_rules(const monitor_bpf_rule_t *rules, uint8_t count)`

Installs up to `MAX_BPF_RULES` BPF rules. `count` entries are copied from `rules`. If `count > MAX_BPF_RULES`, excess rules are silently ignored. Call `monitor_clear_bpf_rules()` to disable all rules.

**Note:** Offsets are relative to the start of the **802.11 MAC header**, not the promisc buffer (which includes driver metadata before the MAC header). The library handles this offset internally.

---

#### `void monitor_clear_bpf_rules(void)`

Removes all active BPF rules. All frames passing the other filters will be forwarded.

---

### Statistics

#### `int monitor_get_channel_stats(uint8_t channel, channel_stats_t *out)`

Copies the per-channel statistics for `channel` into `*out`. Returns `0` on success, `-1` if `channel` is out of range. Statistics are accumulated since the last `monitor_reset_channel_stats()` or `monitor_start()`.

**Computing RSSI mean and standard deviation from the struct:**
```c
channel_stats_t s;
monitor_get_channel_stats(6, &s);
float mean = (float)s.rssi_sum / (float)s.frames_total;
float var  = (float)s.rssi_m2  / (float)(s.frames_total - 1); // Welford
float std  = sqrtf(var);
```

---

#### `void monitor_reset_channel_stats(void)`

Resets all per-channel statistics to zero.

---

#### `void monitor_get_stats(uint32_t *captured, uint32_t *dropped_ring, uint32_t *dropped_pool, uint32_t *peak_pool, uint32_t *dropped_block)`

Reads the global capture counters. Any parameter may be `NULL` to skip that counter. Values are read directly from the `volatile` globals — no lock is taken.

---

#### `void monitor_reset_stats(void)`

Resets all global capture counters to zero.

---

### Capture Callback

#### `void monitor_set_capture_callback(monitor_capture_cb_t cb)`

Registers a per-frame callback invoked from `monitor_task` for each frame that passes all filters. The callback receives a pointer into the pool buffer (valid only for the duration of the callback), the frame length, channel, RSSI, and nanosecond timestamp.

Set to `NULL` to disable.

---

## Output Format Details

### pcapng (default, `MONITOR_FMT_PCAPNG`)

Writes a compliant pcapng stream with nanosecond timestamps:

- **SHB** (Section Header Block) written once at `monitor_start()`
- **IDB** (Interface Description Block) written once, with options:
  - `if_tsresol = 9` (timestamp resolution = 10⁻⁹ s = 1 ns)
  - `if_fcslen = 4` (802.11 FCS included)
  - Link type: `LINKTYPE_IEEE802_11_RADIOTAP` (127)
- **EPB** (Enhanced Packet Block) per captured frame, containing:
  - 24-byte radiotap header (channel, RSSI, rate, flags)
  - Full 802.11 frame including FCS

The stream is valid pcapng and can be opened directly in Wireshark or processed with `tshark`/`pcapng` libraries.

### pcap (`MONITOR_FMT_PCAP`)

Writes a standard libpcap stream:

- Global header: magic `0xA1B2C3D4`, version 2.4, µs resolution
- Per-packet record with `ts_sec` / `ts_usec` from the nanosecond timestamp
- Link type: `LINKTYPE_IEEE802_11_RADIOTAP` (127)
- 24-byte radiotap header prepended to each frame

### raw (`MONITOR_FMT_CAP`)

Writes bare 802.11 frames with no file header and no radiotap encapsulation. Each frame is the raw promisc buffer content. Useful for custom parsers or streaming to a secondary processor.

---

## Radiotap Header Layout

The library constructs a fixed 24-byte radiotap header. Fields present (in order):

| Offset | Length | Field | Source |
|--------|--------|-------|--------|
| 0 | 1 | Revision | 0 |
| 1 | 1 | Pad | 0 |
| 2 | 2 | Header length | 24 (LE) |
| 4 | 4 | Present flags bitmap | (see below) |
| 8 | 8 | Timestamp (µs) | `ts_ns / 1000` |
| 16 | 2 | Channel frequency (MHz) | Derived from channel number |
| 18 | 2 | Channel flags | 802.11b/g/n flag |
| 20 | 1 | RSSI (dBm) | From `rx_pkt_info` |
| 21 | 1 | Signal quality | 0 (not available) |
| 22 | 2 | RX flags | 0 |

Present bits: `TSFT | FLAGS | CHANNEL | DBM_ANTSIGNAL | RX_FLAGS`

---

## Pool Sizing and Back-pressure

The library uses two pools as explicit resource limits. When either is exhausted, frames are dropped and the corresponding counter is incremented.

**Packet pool (`PACKET_POOL_SIZE = 64`)** is the first bottleneck. The promisc callback holds a pool slot until the frame is processed by `monitor_task` and forwarded to the writer queue. Under sustained load at high data rates, the pool drains before the ring fills. Increasing `PACKET_POOL_SIZE` (and correspondingly increasing SRAM) extends burst tolerance.

**Block pool (`BLOCK_POOL_SIZE = 32`)** limits the writer task's in-flight output batches. The block pool is released after the sink callback returns. If the sink is slow (e.g., UART at 115200 bps), the block pool drains and `stats_dropped_block` increments. Increase `BLOCK_POOL_SIZE` or use a faster sink.

**Recommended sizing guidance:**

| Scenario | `PACKET_POOL_SIZE` | `BLOCK_POOL_SIZE` | Sink |
|----------|--------------------|-------------------|------|
| Quiet channel scan | 32 | 16 | Any |
| Dense AP environment | 64 (default) | 32 (default) | USB CDC or VFS |
| High-throughput data capture | 128 | 64 | USB HS or fast VFS |

Note: `RING_SIZE = 1024` is larger than `PACKET_POOL_SIZE` by design — the ring acts as a pointer queue, not a data buffer, and its overhead is only 4 KB.

---

## Usage Examples

### Basic pcapng capture to UART

```c
#include "monitor.h"
#include "serial_api.h"

static serial_t g_uart;

static void uart_sink(const void *data, size_t len)
{
    serial_send_blocked(&g_uart, (uint8_t *)data, (uint32_t)len, 5000);
}

void capture_demo(void)
{
    serial_init(&g_uart, UART_TX, UART_RX);
    serial_baud(&g_uart, 3000000);  // 3 Mbps for USB-UART adapters

    monitor_set_output_sink(uart_sink);
    monitor_set_fixed_channel(6);   // Lock to channel 6
    monitor_set_filter(FILTER_MANAGEMENT);
    monitor_set_rssi_threshold(-80); // Ignore very weak frames

    monitor_start();
    // Capture runs until monitor_stop() is called
}
```

### MAC allowlist with BPF

```c
// Only capture data frames from a specific client MAC
static const uint8_t target_mac[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

// BPF: match Data frames (FC byte 0: bits[3:2] == 10b → type=data)
static const monitor_bpf_rule_t bpf[] = {
    { .offset = 0, .mask = 0x0C, .value = 0x08 },
};

monitor_set_mac_filter_mode(MAC_FILTER_ALLOWLIST);
monitor_add_mac_filter(target_mac);
monitor_set_bpf_rules(bpf, 1);
monitor_set_output_sink(my_sink);
monitor_start();
```

### Adaptive channel hopping with traffic analysis

```c
static void frame_cb(const uint8_t *frame, uint32_t len,
                     uint8_t channel, int8_t rssi, uint64_t ts_ns)
{
    printf("[ch%2u | %4d dBm | %llu ns] %u bytes\n",
           channel, rssi, (unsigned long long)ts_ns, len);
}

monitor_set_hopping();              // Adaptive dwell
monitor_set_capture_callback(frame_cb);
monitor_set_output_sink(my_sink);   // Still writing pcapng
monitor_start();

vTaskDelay(pdMS_TO_TICKS(30000));

// Print per-channel summary
for (uint8_t ch = 1; ch <= 13; ch++) {
    channel_stats_t s;
    if (monitor_get_channel_stats(ch, &s) == 0 && s.frames_total > 0) {
        printf("ch%2u: %lu frames, RSSI [%d, %d] dBm, EMA traffic: %lu\n",
               ch,
               (unsigned long)s.frames_total,
               s.rssi_min, s.rssi_max,
               (unsigned long)s.traffic_ema);
    }
}

monitor_stop();
```

### Runtime statistics monitoring

```c
void print_capture_health(void)
{
    uint32_t captured, dropped_ring, dropped_pool, peak_pool, dropped_block;
    monitor_get_stats(&captured, &dropped_ring, &dropped_pool,
                      &peak_pool, &dropped_block);

    float loss_pct = 0.0f;
    uint32_t total = captured + dropped_ring + dropped_pool + dropped_block;
    if (total > 0) loss_pct = 100.0f * (total - captured) / (float)total;

    printf("Captured: %lu | Ring drops: %lu | Pool drops: %lu "
           "| Block drops: %lu | Peak pool: %lu/%u | Loss: %.1f%%\n",
           (unsigned long)captured,
           (unsigned long)dropped_ring,
           (unsigned long)dropped_pool,
           (unsigned long)dropped_block,
           (unsigned long)peak_pool,
           PACKET_POOL_SIZE,
           loss_pct);
}
```

---

## Interaction with the Injector Library

The monitor and injector libraries share the single RTL8721Dx radio. Key constraints:

- **Simultaneous operation:** The radio can be in promiscuous mode while injecting — the Realtek driver supports concurrent promisc and TX on the same interface. Monitor and inject can run together.
- **Channel conflict:** If the injector switches channels, the monitor's fixed channel setting is overridden at the hardware level. Use `INJ_FLAG_FIXED_CHANNEL` on injectors that must not affect the monitor's channel, or coordinate channel assignment at the application level via a shared `wifi_radio.h` arbitration layer.
- **Receive during TX:** Frames transmitted by the injector will appear in the monitor's promisc stream. Apply a MAC denylist entry for your own transmitter MAC to suppress loopback frames if needed.
