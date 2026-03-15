# RTL-base

**RTL-base** turns your AmebaDplus (RTL8721Dx) board into a professional dual‑band Wi‑Fi monitor and packet injector.

---

## Features

### Monitor Mode (pcapng over UART)
- Full‑band channel hopping (2.4 GHz & 5 GHz) – cycles through 1‑13 and 36‑165.
- Captures **all** 802.11 frames (data, management, control) via promiscuous mode.
- Emits a **valid pcapng stream** containing:
  - Section Header Block (SHB)
  - Interface Description Block (IDB) with linktype 127 (radiotap)
  - Enhanced Packet Blocks (EPB) with **radiotap headers** (channel, RSSI, data rate) + raw frame
- High‑resolution timestamps using the **DWT cycle counter** (nanoseconds).
- Lock‑free ring buffer and packet pool – safe for interrupt context, zero packets lost under moderate load.
- Configurable UART baud rate (default 2 Mbaud) for real‑time streaming.

### Packet Injector (Raw Frame Scheduler)
- Create up to 16 **named injectors** with independent parameters:
  - Channel (auto‑switched unless fixed)
  - Transmission interval (nanoseconds)
  - Max packets / retry limit
  - Data rate (1 Mbps … MCS7, matching Ameba rate definitions)
  - TX power (dBm, if supported)
  - Flags: short GI, aggregation, fixed channel, etc.
- Scheduler runs as an RTOS task or can be driven by a hardware timer.
- Full control API: start, stop, activate/deactivate, modify parameters **on the fly**.
- Weak platform hooks for time/timer – easy to adapt to any hardware.

---

## Hardware Requirements

- An **AmebaDplus** board (e.g. bw20, RTL8721Dx, RTL8711, etc.)
- A USB‑to‑UART adapter (3.3V logic) connected to:
  - TX → `_PB_5` (board's TX pin)
  - RX → `_PB_4` (board's RX pin)
- Optional: A second UART for debug messages (if you modify the code).

---

## Prerequisites
- [Ameba RTOS SDK](https://github.com/Ameba-AIoT/ameba-rtos)

---

## License

This project is licensed under the GPLv2 – see [LICENSE](LICENSE) for details.
