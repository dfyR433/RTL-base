# Timer Library — `timer.h` / `timer.c`

**Target:** RTL8721Dx (Ameba) — KM4 core (ARM Cortex-M55) @ 345 MHz  
**Resolution:** ~3 ns (DWT tier) / ~10 ns (SysTick tier) / 1 µs (GTimer coarse)  
**Thread safety:** All public functions are ISR-safe and re-entrant unless noted.

---

## Overview

The timer library provides a composite nanosecond timestamp by fusing two independent clocks:

- **Ameba GTimer** (`gtimer_t`, TIMER0 by default) — a 32-bit hardware counter clocked at 1 MHz, giving 1 µs resolution and a ~71-minute free-running range. Used as the stable, drift-anchored coarse reference.
- **ARM DWT CYCCNT** — the Cortex-M debug cycle counter, clocked directly from the CPU. At 345 MHz this yields ~2.9 ns per tick. Used to interpolate within each 1 µs GTimer tick.

If DWT is unavailable (e.g., a locked-out debug unit), the library falls back automatically to **SysTick** for sub-microsecond interpolation, and further falls back to coarse GTimer-only timestamps if SysTick is also unusable.

A **phase calibration** step aligns the DWT/SysTick sub-microsecond fraction to the GTimer tick edge, correcting for the fixed phase offset between the two clock domains. A lightweight **FreeRTOS watchdog task** samples DWT every `TIMER_WRAP_WATCHDOG_MS` milliseconds to extend the 32-bit cycle counter to 64 bits without OS dependencies in the hot path.

### Timestamp Tiers

| Tier | Source | Resolution | Availability |
|------|--------|-----------|-------------|
| Fine (DWT) | GTimer + DWT CYCCNT | ~3 ns | After `timer_calibrate()`, DWT enabled |
| Fine (SysTick) | GTimer + SysTick | ~10 ns | After `timer_calibrate()`, DWT absent |
| Coarse | GTimer only | 1 µs | Always, after `timer_init()` |

---

## Initialization Sequence

Always call in this order:

```c
timer_init();        // Start GTimer and DWT; detect hardware tier
timer_calibrate();   // Phase-align DWT/SysTick to GTimer; spawn watchdog task
bool ok = timer_wrap_test();  // Optional: verify watchdog fires correctly
```

`timer_init()` is idempotent — safe to call multiple times; re-entry is no-op.  
`timer_calibrate()` may be called multiple times to recalibrate (e.g., after a CPU frequency change).

---

## Configuration Macros

All macros are overridable at compile time via `-D` flags.

| Macro | Default | Description |
|-------|---------|-------------|
| `TIMER_GTIMER_IDX` | `TIMER0` | Hardware GTimer index (0–3) |
| `TIMER_CAL_SAMPLES` | `8` | Phase calibration sample count. Must be in `[4, 64]`. More samples reduce phase noise but increase calibration time. |
| `TIMER_FINE_RESOLUTION_NS` | `3` | Reported fine resolution (informational only; actual resolution depends on CPU frequency). |
| `TIMER_COARSE_RESOLUTION_NS` | `1000` | Reported coarse resolution. |
| `TIMER_SPIN_MIN_NS` | `100` | Minimum duration accepted by spin-wait functions. Shorter requests return immediately. |
| `TIMER_SPIN_MAX_NS` | `10,000,000` | Maximum duration accepted by spin-wait functions (~10 ms). Longer requests return immediately as a safety guard. |
| `TIMER_WRAP_WATCHDOG_MS` | `4000` | DWT watchdog task period. Must satisfy `TIMER_WRAP_WATCHDOG_MS * 2 < 10000`. At 200 MHz, DWT wraps every ~21.5 s so 4 s is conservative. |
| `TIMER_CAL_MAX_MASK_CYCLES` | `20` | Maximum CPU cycles tolerated as interrupt jitter during calibration sampling. |

---

## API Reference

### Initialization

#### `void timer_init(void)`

Initializes the GTimer hardware, enables the DWT cycle counter via `CoreDebug->DEMCR`, and probes SysTick as a fallback. Computes `s_cycles_per_us` from `SystemCoreClock` (defaults to 200 MHz if `SystemCoreClock` returns zero).

Must be called from a task context (uses `taskENTER_CRITICAL`). Subsequent calls are no-ops.

---

#### `void timer_calibrate(void)`

Performs multi-sample phase alignment between DWT (or SysTick) and the GTimer 1 µs tick edge.

For each of `TIMER_CAL_SAMPLES` iterations, the function:
1. Busy-waits for a GTimer tick edge (rising edge of µs counter).
2. Atomically captures the DWT/SysTick sub-microsecond count at that edge.
3. Records the phase offset `phase = count % cycles_per_us`.

The mean phase across all samples is stored as `s_dwt_phase` / `s_systick_phase`. A coarse drift estimate in PPM is computed from the phase drift across the sample span.

On first call, spawns `dwt_watchdog_task` (stack: 256 words, priority `tskIDLE_PRIORITY + 2`). If task creation fails, DWT is disabled and the library falls back to coarse GTimer timestamps.

**Blocking:** ~`TIMER_CAL_SAMPLES` µs of busy-wait plus one `xTaskCreate`.

---

#### `bool timer_wrap_test(void)`

Self-test for the DWT wrap-watchdog. Artificially sets `s_dwt_prev = 0xFFFFFFFF` to force a simulated wrap, then waits up to `TIMER_WRAP_WATCHDOG_MS * 2` ms for the watchdog to detect it. Rolls back the false wrap increment if successful.

Returns `true` if the watchdog fired within the deadline, `false` otherwise.

**Note:** This function sleeps via `vTaskDelay` and must not be called from an ISR or a task with hard real-time constraints.

---

### Timestamp Accessors

#### `uint64_t timer_get_time_ns(void)`

Returns the current time in nanoseconds using the coarse GTimer only (`gtimer_us() * 1000`). Resolution is 1 µs. Always available after `timer_init()`. ISR-safe.

---

#### `uint64_t timer_get_time_ns_fine(void)`

Returns the current time in nanoseconds using the highest available tier:

- **DWT tier:** Performs a double-read of DWT around a GTimer sample (`u0, us, u1`) with up to 4 retries to minimize interrupt-induced jitter. Midpoint DWT value is phase-corrected and converted to nanoseconds, then added to `us * 1000`.
- **SysTick tier:** Identical algorithm using `SYSTICK_UP()` (counts *up* from the reload edge) in place of DWT.
- **Coarse fallback:** Returns `gtimer_us() * 1000`.

ISR-safe. No OS calls. Bounded execution time.

---

#### `uint64_t timer_get_time_ns_isr(void)`

Alias for `timer_get_time_ns_fine()`. Provided for clarity at call sites that must document ISR-safe usage.

---

#### `uint64_t timer_get_time_us(void)`

Returns the current GTimer value in microseconds. Equivalent to `gtimer_read_us()`.

---

#### `uint64_t timer_align_ns(uint64_t period_ns)`

Returns the next timestamp that is an exact multiple of `period_ns`, at or after the current time. Useful for phase-locking burst starts to a grid:

```c
// Start the first burst on the next 10 ms boundary
uint64_t aligned = timer_align_ns(10000000ULL);
```

Returns `timer_get_time_ns_fine()` unchanged if `period_ns == 0`.

---

### Cycle Counter

#### `uint32_t timer_get_cycles32(void)`

Returns the raw 32-bit DWT CYCCNT value. Returns `0` if DWT is unavailable. ISR-safe.

---

#### `uint64_t timer_get_cycles64(void)`

Returns a 64-bit cycle count by combining the hardware 32-bit CYCCNT with `s_dwt_hi` (the overflow word maintained by the watchdog task). Uses a lock-free double-read retry to guard against a watchdog increment racing between the two reads:

```c
do { hi1 = s_dwt_hi; lo = DWT_NOW(); hi2 = s_dwt_hi; } while (hi1 != hi2);
return ((uint64_t)hi1 << 32) | lo;
```

Returns `0` if DWT is unavailable. ISR-safe.

---

### Unit Conversion

Both functions return `0` if called before `timer_init()`.

#### `uint64_t timer_ns_to_cycles(uint64_t ns)`

Converts a nanosecond duration to CPU cycles using precomputed integer ratios. Avoids runtime division:

```
cycles = (ns / 1000) * cycles_per_us
       + (ns % 1000 * cycles_per_us + 500) / 1000
```

---

#### `uint64_t timer_cycles_to_ns(uint64_t cycles)`

Converts CPU cycles to nanoseconds:

```
ns = (cycles / cycles_per_us) * 1000
   + (cycles % cycles_per_us) * 1000 / cycles_per_us
```

---

### Precision Busy-Wait

Both functions are no-ops if the requested duration is outside `[TIMER_SPIN_MIN_NS, TIMER_SPIN_MAX_NS]`.

#### `void timer_spin_until_ns(uint64_t target_abs_ns)`

Busy-spins until the fine timestamp reaches `target_abs_ns`. Uses signed 32-bit wrap-safe arithmetic against DWT or SysTick to avoid reading the GTimer on every iteration (GTimer reads are comparatively expensive at 1 µs resolution):

```c
uint32_t deadline = base + timer_ns_to_cycles(delta_ns);
while ((int32_t)(deadline - DWT_NOW()) > 0)
    __asm__ volatile("isb" ::: "memory");
```

Falls back to `timer_get_time_ns()` polling if neither DWT nor SysTick is calibrated.

Returns immediately if `target_abs_ns` is already in the past, or if the delta exceeds `TIMER_SPIN_MAX_NS`.

---

#### `void timer_spin_for_ns(uint64_t duration_ns)`

Convenience wrapper: `timer_spin_until_ns(timer_get_time_ns_fine() + duration_ns)`.

---

### Diagnostics

| Function | Returns | Notes |
|----------|---------|-------|
| `timer_is_ready()` | `bool` | `true` after `timer_init()` |
| `timer_is_calibrated()` | `bool` | `true` after successful `timer_calibrate()` |
| `timer_get_resolution_ns()` | `uint32_t` | `TIMER_FINE_RESOLUTION_NS` if calibrated, else `TIMER_COARSE_RESOLUTION_NS` |
| `timer_get_resolution_source()` | `const char *` | `"dwt"`, `"systick"`, or `"gtimer"` |
| `timer_get_cpu_freq_hz()` | `uint32_t` | CPU frequency used for cycle math |
| `timer_get_dwt_phase()` | `uint32_t` | Calibrated DWT phase offset (cycles) |
| `timer_get_wrap_count()` | `uint32_t` | Number of 32-bit DWT overflows detected |
| `timer_get_drift_ppm()` | `int32_t` | Estimated DWT/GTimer relative drift across the calibration sample span |

---

## Design Notes

### Why DWT over a second hardware timer?

A second hardware timer would require interrupt/critical-section coordination to read atomically. DWT CYCCNT is read with a single 32-bit load — no race conditions in the sub-microsecond interpolation path. The tradeoff is that DWT may be disabled by a debug lock, hence the SysTick fallback.

### Phase correction

Without calibration, `(DWT_NOW() % cycles_per_us)` could be an arbitrary offset from the true µs boundary. The `s_dwt_phase` correction subtracts this fixed offset so the interpolated sub-microsecond value is correctly aligned to GTimer ticks.

### Drift PPM interpretation

`timer_get_drift_ppm()` measures phase scatter across the `TIMER_CAL_SAMPLES` calibration samples, not a true long-term frequency error. It is a useful sanity check (|drift| > ~100 PPM during calibration suggests severe interrupt jitter), but it should not be used as a calibrated correction factor without a much longer measurement window.

### 64-bit overflow

At 345 MHz, DWT CYCCNT overflows every ~12.4 seconds. The watchdog task samples every `TIMER_WRAP_WATCHDOG_MS` (4 s by default), which guarantees at most one overflow between samples. The `timer_get_cycles64()` double-read retry is correct as long as the watchdog period is less than the hardware wrap period (~12.4 s at 345 MHz, ~21.5 s at 200 MHz).

---

## Usage Example

```c
#include "timer.h"

void app_main(void)
{
    timer_init();
    timer_calibrate();

    // Verify watchdog (optional, call once at startup)
    if (!timer_wrap_test()) {
        printf("WARNING: DWT watchdog not firing\n");
    }

    printf("Timer source: %s, resolution: %lu ns\n",
           timer_get_resolution_source(),
           (unsigned long)timer_get_resolution_ns());

    // Fine timestamp
    uint64_t t0 = timer_get_time_ns_fine();
    do_work();
    uint64_t t1 = timer_get_time_ns_fine();
    printf("Work took %llu ns\n", (unsigned long long)(t1 - t0));

    // Phase-aligned spin-wait
    uint64_t next = timer_align_ns(1000000ULL); // next 1 ms boundary
    timer_spin_until_ns(next);

    // Cycle counting
    uint32_t c0 = timer_get_cycles32();
    hot_path();
    uint32_t c1 = timer_get_cycles32();
    printf("Hot path: %lu cycles (~%llu ns)\n",
           (unsigned long)(c1 - c0),
           (unsigned long long)timer_cycles_to_ns(c1 - c0));
}
```
