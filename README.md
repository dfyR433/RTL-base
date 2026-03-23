# RTL-base

**RTL-base** turns your AmebaDplus (RTL8721Dx & RTL8711Dx) board into a dual‑band Wi‑Fi monitor and packet injector.

---

## Warning

This project is still being tested.

---

## Features

### Monitor Mode (pcapng over UART)
- Full‑band channel hopping (2.4 GHz & 5 GHz) – cycles through 1‑13 and 36‑165.
- Captures **all** 802.11 frames (data, management, control) via promiscuous mode.
- Emits a **valid pcapng stream** containing:
  - Section Header Block (SHB)
  - Interface Description Block (IDB) with linktype 127 (radiotap)
  - Enhanced Packet Blocks (EPB) with **radiotap headers** (channel, RSSI, data rate) + raw frame
- Lock‑free ring buffer and packet pool – safe for interrupt context, zero packets lost under moderate load.

### Packet Injector (Raw Frame Scheduler)
- Create up to 16 **named injectors** with independent parameters:
  - Channel (auto‑switched unless fixed)
  - Transmission interval (nanoseconds)
  - Max packets
  - Data rate
  - TX power (dBm)
  - Flags: short GI, aggregation, fixed channel, etc.
- Full control API: set, clear, activate/deactivate, modify parameters **on the fly**.

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
