/**
 * @file  timer.h
 * @brief ns-precision timestamp — RTL8721Dx / KM4 (Cortex-M55)
 */

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Configuration
 * ========================================================================= */
#ifndef TIMER_GTIMER_IDX
#  define TIMER_GTIMER_IDX            TIMER0
#endif

#ifndef TIMER_CAL_SAMPLES
#  define TIMER_CAL_SAMPLES           8u
#endif

#define TIMER_CAL_MAX_MASK_CYCLES     20u
#define TIMER_COARSE_RESOLUTION_NS    1000ULL

#define TIMER_FINE_RESOLUTION_NS      3ULL

#ifndef TIMER_SPIN_MAX_NS
#  define TIMER_SPIN_MAX_NS           10000000ULL
#endif

#ifndef TIMER_SPIN_MIN_NS
#  define TIMER_SPIN_MIN_NS           100ULL
#endif

#define TIMER_WRAP_WATCHDOG_MS        4000u

/* =========================================================================
 * Initialisation
 * ========================================================================= */
void timer_init(void);
void timer_calibrate(void);
bool timer_wrap_test(void);

/* =========================================================================
 * Timestamp accessors
 * ========================================================================= */
uint64_t timer_get_time_ns(void);
uint64_t timer_get_time_ns_fine(void);
uint64_t timer_get_time_ns_isr(void);

uint64_t timer_get_time_us(void);
uint64_t timer_align_ns(uint64_t period_ns);

/* =========================================================================
 * DWT cycle counter  (DWT tier only; returns 0 when DWT unavailable)
 * ========================================================================= */
uint32_t timer_get_cycles32(void);
uint64_t timer_get_cycles64(void);

/* =========================================================================
 * Unit conversion  (precomputed integer ratios, zero runtime division)
 * Returns 0 if called before timer_init().
 * ========================================================================= */
uint64_t timer_ns_to_cycles(uint64_t ns);
uint64_t timer_cycles_to_ns(uint64_t cycles);

/* =========================================================================
 * Precision busy-wait
 * ========================================================================= */
void timer_spin_until_ns(uint64_t target_abs_ns);
void timer_spin_for_ns(uint64_t duration_ns);

/* =========================================================================
 * Diagnostics
 * ========================================================================= */
bool     timer_is_ready(void);
bool     timer_is_calibrated(void);
uint32_t timer_get_resolution_ns(void);

const char *timer_get_resolution_source(void);

uint32_t timer_get_cpu_freq_hz(void);
uint32_t timer_get_dwt_phase(void);
uint32_t timer_get_wrap_count(void);
int32_t  timer_get_drift_ppm(void);

#ifdef __cplusplus
}
#endif
#endif /* TIMER_H */
