# RTL-base

A real-time library for high-precision Wi-Fi packet injection and a promiscuous-mode capture engine that streams 802.11 frames over UART as standard capture files. Designed for embedded systems running Ameba-rtos (Ameba RTL8721Dx).

---

## Overview

This project provides two independent but co‑usable components:

* **Wi-Fi Packet Injector Manager** – schedule raw 802.11 frames with nanosecond precision. Supports multiple named injectors, periodic or single‑shot operation, hardware/software retries, dynamic parameter updates, and latency compensation.
* **Wi-Fi Monitor Mode Engine** – capture Wi-Fi frames in promiscuous mode and stream them over UART as libpcap (`pcap`), raw `cap`, or pcapng files. Offers per‑frame filtering (RSSI, MAC, frame type) and channel hopping.

Both modules are thread‑safe, SMP‑aware, and optimised for low‑overhead operation on resource‑constrained RTOS targets.

---

## Features

### Packet Injector Manager
- **Multiple independent injectors** – up to 16 named instances.
- **High‑precision scheduling** – nanosecond resolution with spin‑guard and latency compensation.
- **Periodic & single‑shot** – `interval_ns = 0` for one‑shot, any positive value for periodic.
- **Dynamic updates** – change rate, channel, power, interval, payload, etc. at runtime.
- **Retries** – configurable hardware retry limit and software retry fallback.
- **Channel & power control** – per‑injector channel selection, dBm‑to‑hardware mapping callback.
- **Statistics** – per‑injector and global counters for packets, errors, and retries.
- **Thread‑safe** – all public functions protected by a mutex; generation‑counter prevents ABA races.

### Monitor Mode Engine
- **Multiple output formats** – libpcap (with/without radiotap) and pcapng (default).
- **Frame‑type filters** – data, management, control, or specific subtypes (beacon, probe request/response).
- **RSSI threshold** – drop frames below a configurable dBm level.
- **MAC filtering** – allowlist or denylist based on DA, SA, or BSSID (up to 16 entries).
- **Channel control** – fixed channel or automatic hopping across a predefined list.
- **Zero side‑band data** – UART carries only the capture stream; statistics are read in‑memory.
- **Lock‑free data path** – SPSC ring buffer and packet pool with atomic operations, safe for ISR context.

---

## Building & Dependencies

This code is intended for **Ameba RTL8721Dx**‑based boards running FreeRTOS with the Realtek Wi‑Fi driver. It relies on the following platform‑specific symbols (provided by your BSP or application):

| Symbol | Description |
|--------|-------------|
| `init_highres_timer` / `read_highres_timer` | High‑resolution timer (injection scheduler) |
| `wifi_set_channel` | Change Wi‑Fi channel |
| `wifi_send_raw_frame` | Send a raw 802.11 frame |
| `wifi_set_tx_power_percentage` | Set TX power percentage (optional) |
| `monitor_uart` / `serial_putc` / `serial_writable` | UART handle and byte‑transmit functions (monitor) |
| `wifi_promisc_enable` / `wifi_set_channel` / `wifi_get_tsf` | Promiscuous mode control (monitor) |

Override the default configuration macros by defining them before including the respective headers.

## Quick Start

```c
#include "inject.h"
#include "monitor.h"

static injectorManager *g_mgr = NULL;

void app_main(void) {
    // --- Setup injector ---
    injector_set_timer_freq_hz(80000000UL);          // 80 MHz hardware timer
    g_mgr = injectorManager_create();
    injectorManager_setWlanIndex(g_mgr, 0);

    uint8_t beacon[] = { /* 802.11 beacon frame */ };
    injectorManager_setInjector(g_mgr, "beacon", beacon, sizeof(beacon),
                                6,       // channel
                                102400,  // interval (µs) → auto‑converted to ns
                                0,       // unlimited packets
                                3);      // hw retries

    // --- Setup monitor ---
    monitor_set_filter(FILTER_BEACON);
    monitor_set_rssi_threshold(-70);
    monitor_set_fixed_channel(6);           // match injector channel

    // --- Start both ---
    injectorManager_startSchedulerTask(g_mgr, 5);
    injectorManager_activateInjector(g_mgr, "beacon");

    monitor_start();                        // begins capturing and streaming

    // --- Application loop ---
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        // Print stats (injector & monitor) via your own debug channel
        InjectorInfo info;
        injectorManager_getInfo(g_mgr, "beacon", &info);
        printf("injector: sent=%u errors=%u retries=%u\n",
               info.packets_sent, info.tx_errors, info.tx_retries);

        uint32_t cap, dr, dp, pk;
        monitor_get_stats(&cap, &dr, &dp, &pk);
        printf("monitor: captured=%u ring_drops=%u pool_drops=%u peak_pool=%u\n",
               cap, dr, dp, pk);
    }

    // --- Shutdown (not reached in this example) ---
    monitor_stop();
    injectorManager_stopSchedulerTask(g_mgr);
    injectorManager_destroy(g_mgr);
}
```

### Capturing to Wireshark (Linux)
```bash
stty -F /dev/ttyUSB0 921600 raw -echo
wireshark -k -i <(cat /dev/ttyUSB0)
```

---

## Documentation

Detailed API references, configuration options, and platform requirements are available in the module‑specific documentation:

- [Packet Injector Manager (inject.md)](https://github.com/dfyR433/RTL-base/blob/main/docs/inject.md)
- [Monitor Mode Engine (monitor.md)](https://github.com/dfyR433/RTL-base/blob/main/docs/monitor.md)

---

## Platform Requirements

- [Ameba RTOS SDK](https://github.com/Ameba-AIoT/ameba-rtos)

---

## License

This project is licensed under the **GPLv2**. See the [LICENSE](LICENSE) file for details.
