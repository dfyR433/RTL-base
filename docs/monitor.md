# Wi-Fi Monitor Mode Engine

## Introduction

The **Wi-Fi Monitor Mode Engine** (`monitor.c` / `monitor.h`) captures raw
802.11 frames in promiscuous mode on Ameba (Realtek RTL8721Dx) SoCs and streams
them over UART as a valid capture file.

The UART carries **exactly one thing**: the capture packet stream.
No statistics, no telemetry, no sideband data of any kind.
Statistics are tracked in memory and read via `monitor_get_stats()`.

The output format is chosen once at compile time via `MONITOR_FORMAT`.

---

## Output Formats

| `MONITOR_FORMAT` | File format | Link-layer type | Radiotap |
|---|---|---|---|
| `MONITOR_FMT_CAP` | libpcap | 105 — raw IEEE 802.11 | ✗ |
| `MONITOR_FMT_PCAP` | libpcap | 127 — 802.11 + radiotap | ✓ |
| `MONITOR_FMT_PCAPNG` *(default)* | pcapng | 127 — 802.11 + radiotap | ✓ |

### Selecting a format

```c
// In a shared config header or on the compiler command line:
#define MONITOR_FORMAT  MONITOR_FMT_CAP     // raw 802.11, smallest overhead
#define MONITOR_FORMAT  MONITOR_FMT_PCAP    // libpcap + radiotap
#define MONITOR_FORMAT  MONITOR_FMT_PCAPNG  // pcapng + radiotap (default)
```

```makefile
# Via Makefile / CMakeLists
CFLAGS += -DMONITOR_FORMAT=MONITOR_FMT_PCAP
```

Providing an unrecognised value produces a `#error` at compile time.

---

## Architecture

```
Wi-Fi driver
     │  promisc_callback() — ISR-safe, lock-free
     ▼
  pool_alloc() ──► packet_pool[PACKET_POOL_SIZE]
     │
     ▼
  ring[RING_SIZE]    SPSC, atomic indices
     │
     │  monitor_task — drains ring
     ▼
  write_packet_record()       ← compile-time dispatch
  ┌──────────────────────────────────────────────────┐
  │  CAP    │  pcap_queue_packet()  (no radiotap)    │
  │  PCAP   │  pcap_queue_packet()  (+ radiotap)     │
  │  PCAPNG │  pcapng_queue_epb()   (+ radiotap)     │
  └──────────────────────────────────────────────────┘
     │
     ▼
  writer_queue ──► writer_task ──► uart_write_raw ──► UART
```

### Tasks

| Task | Priority | Stack | Purpose |
|---|---|---|---|
| `monitor_task` | `IDLE+3` | 4096 B | Drain ring, call format writer |
| `writer_task` | `IDLE+2` | 2048 B | Send blocks to UART |
| `hopper_task` | `IDLE+1` | 1024 B | Channel sweep every `HOP_INTERVAL_MS` ms |

---

## Wire Format Details

### `MONITOR_FMT_CAP` and `MONITOR_FMT_PCAP` — libpcap

```
Global header — 24 bytes, written once on monitor_start()
──────────────────────────────────────────────────────────
[0-3]   magic_number   0xA1B2C3D4  (µs timestamps)
[4-5]   version_major  2
[6-7]   version_minor  4
[8-15]  thiszone + sigfigs  0
[16-19] snaplen        PACKET_BUFFER_SIZE
[20-23] network        105  (CAP)  |  127  (PCAP)

Per-packet record — 16-byte header + payload, one per frame
────────────────────────────────────────────────────────────
[0-3]   ts_sec         TSF µs / 1 000 000
[4-7]   ts_usec        TSF µs % 1 000 000
[8-11]  incl_len       captured length (including radiotap if PCAP)
[12-15] orig_len       original length
[16..]  [radiotap +] 802.11 frame
```

### `MONITOR_FMT_PCAPNG` — next-generation pcapng

```
SHB — Section Header Block, 32 bytes, written once
IDB — Interface Description Block, written once
      linktype 127  |  if_tsresol = 9 (ns)  |  if_fcslen = 4
EPB — Enhanced Packet Block, one per captured frame
      timestamp = TSF µs × 1000 (nanoseconds)
      payload   = radiotap header + 802.11 frame
```

### Radiotap fields (`MONITOR_FMT_PCAP` and `MONITOR_FMT_PCAPNG`)

| Field | Width | Content |
|---|---|---|
| TSFT | 8 B | TSF counter (µs) |
| Rate | 1 B | Rate × 2 (500 kbps units) |
| Channel | 4 B | Frequency (MHz) + flags: 0x0080 = 2.4 GHz, 0x0100 = 5 GHz |
| Antenna signal | 1 B | RSSI, dBm (signed) |

---

## Configuration Macros

| Macro | Default | Description |
|---|---|---|
| `MONITOR_FORMAT` | `MONITOR_FMT_PCAPNG` | Output format |
| `PACKET_POOL_SIZE` | 64 | Pre-allocated packet buffers |
| `PACKET_BUFFER_SIZE` | 2346 | Max frame size (bytes) |
| `RING_SIZE` | 1024 | SPSC ring depth |
| `HOP_INTERVAL_MS` | 100 | Channel dwell time (ms) |
| `MONITOR_TASK_STACK` | 4096 | monitor_task stack (bytes) |
| `WRITER_TASK_STACK` | 2048 | writer_task stack (bytes) |
| `HOP_TASK_STACK` | 1024 | hopper_task stack (bytes) |
| `WLAN_IDX` | `STA_WLAN_INDEX` | Wi-Fi interface index |

---

## API Reference

### Lifecycle

#### `void monitor_start(void)`

Enables promiscuous mode, spawns `monitor_task`, `writer_task`, and
`hopper_task`, then writes the session header to UART
(SHB + IDB for pcapng; libpcap global header for cap / pcap).

No-op if already running.

#### `void monitor_stop(void)`

Disables promiscuous mode, deletes all tasks, drains and frees any pending
blocks in the writer queue, and deletes the queue.

The stream ends at the last complete packet.  Calling `monitor_start()`
again opens a new session with a fresh header.

---

### Channel Control

#### `void monitor_set_fixed_channel(uint8_t ch)`

Lock the hopper on one channel (1–13 for 2.4 GHz, 36–165 for 5 GHz).
Takes effect within `HOP_INTERVAL_MS` ms.

#### `void monitor_set_hopping(void)`

Resume scanning all channels in `CHANNEL_LIST`.

#### `uint8_t monitor_get_channel(void)`

Return the channel most recently applied by the hopper (0 before first hop).

---

### Frame-Type Filter

#### `void monitor_set_filter(monitor_filter_t filter)`

| Value | Passes |
|---|---|
| `FILTER_ALL` *(default)* | Every frame |
| `FILTER_DATA` | type = 2 |
| `FILTER_MANAGEMENT` | type = 0 |
| `FILTER_CONTROL` | type = 1 |
| `FILTER_BEACON` | mgmt subtype 8 |
| `FILTER_PROBE_REQ` | mgmt subtype 4 |
| `FILTER_PROBE_RSP` | mgmt subtype 5 |

---

### RSSI Threshold

#### `void monitor_set_rssi_threshold(int8_t min_rssi)`

Drop frames whose driver-reported RSSI is below `min_rssi` (dBm, signed).
Default: `-128` (off — all frames pass).

---

### MAC Address Filter

#### `void monitor_set_mac_filter_mode(mac_filter_mode_t mode)`

| Mode | Behaviour |
|---|---|
| `MAC_FILTER_NONE` *(default)* | No filtering |
| `MAC_FILTER_ALLOWLIST` | Pass only frames where DA, SA, or BSSID is listed |
| `MAC_FILTER_DENYLIST` | Drop frames where DA, SA, or BSSID is listed |

#### `int monitor_add_mac_filter(const uint8_t *mac)`

Add a 6-byte MAC (max 16 entries).  Returns 0 or −1 (full).

#### `int monitor_remove_mac_filter(const uint8_t *mac)`

Remove a MAC.  Returns 0 or −1 (not found).

#### `void monitor_clear_mac_filter(void)`

Remove all entries.

---

### Statistics (in-memory only)

#### `void monitor_get_stats(uint32_t *captured, uint32_t *dropped_ring, uint32_t *dropped_pool, uint32_t *peak_pool)`

| Counter | Meaning |
|---|---|
| `captured` | Frames accepted into the ring |
| `dropped_ring` | Frames lost — ring was full |
| `dropped_pool` | Frames lost — pool was exhausted |
| `peak_pool` | Highest simultaneous pool occupancy |

These counters are never written to UART.

#### `void monitor_reset_stats(void)`

Atomically zeroes all four counters.

---

## Platform Requirements

| Symbol | Signature | Notes |
|---|---|---|
| `monitor_uart` | `serial_t` | Initialised UART handle |
| `serial_putc` | `void(serial_t*, int)` | Transmit one byte |
| `serial_writable` | `int(serial_t*)` | Non-zero when TX FIFO has space |
| `wifi_on` | `int(rtw_mode_t)` | Initialise Wi-Fi stack |
| `wifi_is_running` | `int(uint8_t)` | Check driver state |
| `wifi_promisc_enable` | `void(int, rtw_promisc_para*)` | Toggle promiscuous mode |
| `wifi_set_channel` | `int(uint8_t, uint8_t)` | Change channel |
| `wifi_get_tsf` | `uint64_t(uint8_t)` | Read TSF counter (µs) |

---

## Usage Example

```c
#include "monitor.h"

void capture_main(void) {
    /* Configure filters (all formats) */
    monitor_set_filter(FILTER_BEACON);
    monitor_set_rssi_threshold(-75);
    monitor_set_fixed_channel(6);

    /* Start — session header + packet stream on UART */
    monitor_start();

    /* Periodic stats check — in-memory only, never touches UART */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        uint32_t cap, dr, dp, pk;
        monitor_get_stats(&cap, &dr, &dp, &pk);
        /* log via your own debug channel, not UART */
    }
}
```

### Pipe into Wireshark (Linux)

```bash
stty -F /dev/ttyUSB0 921600 raw -echo
wireshark -k -i <(cat /dev/ttyUSB0)
```

Save the stream to a file and open it later:

```bash
cat /dev/ttyUSB0 > capture.pcapng   # or .pcap / .cap
```

---

## Notes

**UART bandwidth.** At 921600 baud a full-size frame (2346 B + radiotap
≈ 28 B + EPB overhead 56 B) takes ~23 ms to emit.  Use `FILTER_BEACON`
or `FILTER_PROBE_REQ` for high-traffic environments, or increase baud rate.

**Format choice.** `MONITOR_FMT_CAP` has the lowest per-packet overhead
and is best when bandwidth matters and radiotap metadata is not needed.
`MONITOR_FMT_PCAP` adds rate/RSSI/channel without the pcapng overhead.
`MONITOR_FMT_PCAPNG` (default) gives nanosecond timestamps and is the
most future-proof format.

**Thread safety.** Public setters write `volatile` scalars read in the
ISR-level promiscuous callback.  32-bit ARM stores to aligned 32-bit
variables are single-instruction atomic; no mutex is needed for these
fields.  Ring and pool use `__atomic_*` builtins with explicit memory
ordering.

**Channel contention with inject.c.** Both modules call `wifi_set_channel`.
When running both simultaneously, call `monitor_set_fixed_channel()` to
match the injector's channel so they do not race.

---

## License

This project is licensed under the GPLv2.
