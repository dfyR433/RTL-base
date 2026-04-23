/**
 * @file  timer.c
 * @brief ns-precision timestamp — RTL8721Dx / KM4 (Cortex-M55).
 */

#include "timer.h"
#include "timer_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * Compile-time contract assertions
 * ========================================================================= */
_Static_assert(TIMER_WRAP_WATCHDOG_MS * 2u < 10000u,
               "[C5] TIMER_WRAP_WATCHDOG_MS must be < 5 s to be safe at 200 MHz");
_Static_assert(TIMER_CAL_SAMPLES >= 4u && TIMER_CAL_SAMPLES <= 64u,
               "[C6] TIMER_CAL_SAMPLES must be in [4, 64]");
_Static_assert(TIMER_SPIN_MAX_NS < 6000000000ULL,
               "[C4] TIMER_SPIN_MAX_NS must be < 6 s");

/* =========================================================================
 * DWT registers (CMSIS preferred; raw Cortex-M fallback)
 * ========================================================================= */
#if defined(DWT) && defined(CoreDebug)
#  define T_DEMCR        CoreDebug->DEMCR
#  define T_DWT_CTRL     DWT->CTRL
#  define T_DWT_CYCCNT   DWT->CYCCNT
#  define T_DEMCR_TRCENA CoreDebug_DEMCR_TRCENA_Msk
#  define T_DWT_CCENA    DWT_CTRL_CYCCNTENA_Msk
#else
#  define T_DEMCR        (*(volatile uint32_t *)0xE000EDFC)
#  define T_DWT_CTRL     (*(volatile uint32_t *)0xE0001000)
#  define T_DWT_CYCCNT   (*(volatile uint32_t *)0xE0001004)
#  define T_DEMCR_TRCENA (1UL << 24)
#  define T_DWT_CCENA    (1UL << 0)
#endif

/* =========================================================================
 * SysTick registers
 * ========================================================================= */
#if defined(SysTick)
#  define T_SYSTICK_CTRL   SysTick->CTRL
#  define T_SYSTICK_LOAD   SysTick->LOAD
#  define T_SYSTICK_VAL    SysTick->VAL
#else
#  define T_SYSTICK_CTRL   (*(volatile uint32_t *)0xE000E010)
#  define T_SYSTICK_LOAD   (*(volatile uint32_t *)0xE000E014)
#  define T_SYSTICK_VAL    (*(volatile uint32_t *)0xE000E018)
#endif
#define T_SYSTICK_ENABLE  (1UL << 0)
#define T_SYSTICK_CLKSRC  (1UL << 2)

/* =========================================================================
 * Module state
 * ========================================================================= */
static gtimer_t  s_gtimer;

static volatile bool s_ready        = false;
static volatile bool s_calibrated   = false;
static volatile bool s_calibrating  = false;
static volatile bool s_dwt_ok       = false;
static volatile bool s_systick_ok   = false;

static uint32_t  s_cpu_freq_hz      = 0u;
static uint32_t  s_cycles_per_us    = 0u;

static uint32_t  s_fine_resolution_ns = (uint32_t)TIMER_COARSE_RESOLUTION_NS;

static uint32_t  s_dwt_phase        = 0u;
static int32_t   s_drift_ppm        = 0;
static volatile uint32_t s_dwt_hi   = 0u;
static volatile uint32_t s_dwt_prev = 0u;
static volatile uint32_t s_wrap_test_count = 0u;
static volatile uint32_t s_overrun_count   = 0u;
static bool      s_watchdog_started = false;

static uint32_t  s_systick_phase    = 0u;
static uint32_t  s_cycles_per_tick  = 0u;

static volatile uint32_t s_gtimer_hi   = 0u;
static volatile uint32_t s_gtimer_prev = 0u;

/* =========================================================================
 * Low-level helpers
 * ========================================================================= */

static inline uint32_t DWT_NOW(void)   { return (uint32_t)T_DWT_CYCCNT; }

static inline uint32_t SYSTICK_UP(void)
{
    return s_cycles_per_tick - (uint32_t)T_SYSTICK_VAL;
}

static inline uint64_t gtimer_us_64(void)
{
    uint32_t lo, hi1, hi2;
    do {
        hi1 = s_gtimer_hi;
        lo  = (uint32_t)gtimer_read_us(&s_gtimer);
        hi2 = s_gtimer_hi;
    } while (hi1 != hi2);
    return ((uint64_t)hi1 << 32) | (uint64_t)lo;
}

/* =========================================================================
 * DWT + GTimer wrap-watchdog task
 * ========================================================================= */
static void dwt_watchdog_task(void *param)
{
    (void)param;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        TickType_t before = xTaskGetTickCount();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TIMER_WRAP_WATCHDOG_MS));
        TickType_t slept = xTaskGetTickCount() - before;

        bool overrun = (slept > pdMS_TO_TICKS(TIMER_WRAP_WATCHDOG_MS * 6u / 5u));

        taskENTER_CRITICAL();
        uint32_t now_dwt = DWT_NOW();
        if (now_dwt < s_dwt_prev) {
            s_dwt_hi++;
            s_wrap_test_count++;
        }
        if (overrun) {
            s_calibrated = false;
            s_overrun_count++;
        }
        s_dwt_prev = now_dwt;
        taskEXIT_CRITICAL();

        taskENTER_CRITICAL();
        uint32_t now_gt = (uint32_t)gtimer_read_us(&s_gtimer);
        if (now_gt < s_gtimer_prev) {
            s_gtimer_hi++;
        }
        s_gtimer_prev = now_gt;
        taskEXIT_CRITICAL();
    }
}

/* =========================================================================
 * Initialisation
 * ========================================================================= */
void timer_init(void)
{
    if (s_ready) return;

    s_cpu_freq_hz = SystemCoreClock;
    if (s_cpu_freq_hz == 0u) s_cpu_freq_hz = 200000000u;
    s_cycles_per_us = s_cpu_freq_hz / 1000000u;
    if (s_cycles_per_us == 0u) s_cycles_per_us = 200u;

    s_fine_resolution_ns = (1000u + s_cycles_per_us - 1u) / s_cycles_per_us;

    taskENTER_CRITICAL();
    if (!s_ready) {
        gtimer_init(&s_gtimer, TIMER_GTIMER_IDX);
        gtimer_start(&s_gtimer);

        s_gtimer_prev = (uint32_t)gtimer_read_us(&s_gtimer);
        s_gtimer_hi   = 0u;

        T_DEMCR      |= T_DEMCR_TRCENA;
        T_DWT_CYCCNT  = 0u;
        T_DWT_CTRL   |= T_DWT_CCENA;
        s_dwt_ok      = (bool)(T_DWT_CTRL & T_DWT_CCENA);
        if (s_dwt_ok) {
            s_dwt_prev = DWT_NOW();
        }

        if (!s_dwt_ok) {
            uint32_t ctrl = (uint32_t)T_SYSTICK_CTRL;
            if ((ctrl & T_SYSTICK_ENABLE) && (ctrl & T_SYSTICK_CLKSRC)) {
                uint32_t load     = (uint32_t)T_SYSTICK_LOAD;
                s_cycles_per_tick = load + 1u;
                uint32_t expected = s_cpu_freq_hz / (uint32_t)configTICK_RATE_HZ;
                uint32_t lo       = (expected * 19u) / 20u;
                uint32_t hi       = (expected * 21u) / 20u;
                if (s_cycles_per_tick >= lo && s_cycles_per_tick <= hi) {
                    s_systick_ok = true;
                }
            }
        }

        s_ready = true;
    }
    taskEXIT_CRITICAL();
}

/* =========================================================================
 * Multi-sample phase calibration
 * ========================================================================= */
void timer_calibrate(void)
{
    if (!s_ready) return;
    if (!s_dwt_ok && !s_systick_ok) return;

    taskENTER_CRITICAL();
    if (s_calibrating) {
        taskEXIT_CRITICAL();
        return;
    }
    s_calibrating = true;
    taskEXIT_CRITICAL();

    uint32_t phases[TIMER_CAL_SAMPLES];
    uint32_t us_at_edge[TIMER_CAL_SAMPLES];
    uint32_t cyc_at_edge[TIMER_CAL_SAMPLES];

    for (uint32_t s = 0u; s < TIMER_CAL_SAMPLES; s++) {
        uint32_t us_prev = (uint32_t)gtimer_read_us(&s_gtimer);
        uint32_t up_edge = 0u, us_now = 0u;
        do { us_now = (uint32_t)gtimer_read_us(&s_gtimer); } while (us_now == us_prev);

        taskENTER_CRITICAL();
        up_edge = s_dwt_ok ? DWT_NOW() : SYSTICK_UP();
        taskEXIT_CRITICAL();

        phases[s]     = up_edge % s_cycles_per_us;
        us_at_edge[s] = us_now;
        cyc_at_edge[s] = up_edge;

        taskYIELD();
    }

    uint32_t mean_phase;
    {
        uint32_t lo = phases[0u], hi = phases[0u];
        for (uint32_t s = 1u; s < TIMER_CAL_SAMPLES; s++) {
            if (phases[s] < lo) lo = phases[s];
            if (phases[s] > hi) hi = phases[s];
        }

        if ((hi - lo) > (s_cycles_per_us / 2u)) {
            int64_t shifted_sum = 0;
            uint32_t half = s_cycles_per_us / 2u;
            for (uint32_t s = 0u; s < TIMER_CAL_SAMPLES; s++) {
                int64_t v = (int64_t)phases[s];
                if ((uint32_t)v > half) v -= (int64_t)s_cycles_per_us;
                shifted_sum += v;
            }
            int64_t shifted_mean = shifted_sum / (int64_t)TIMER_CAL_SAMPLES;
            mean_phase = (uint32_t)((shifted_mean + (int64_t)s_cycles_per_us)
                         % (int64_t)s_cycles_per_us);
        } else {
            uint32_t sum = 0u;
            for (uint32_t s = 0u; s < TIMER_CAL_SAMPLES; s++) sum += phases[s];
            mean_phase = sum / TIMER_CAL_SAMPLES;
        }
    }

    if (s_dwt_ok) {
        s_dwt_phase = mean_phase;
    } else {
        s_systick_phase = mean_phase;
    }

    if (s_dwt_ok) {
        uint64_t us_last  = (uint64_t)us_at_edge[TIMER_CAL_SAMPLES - 1u];
        uint64_t us_first = (uint64_t)us_at_edge[0u];
        if (us_last > us_first) {
            uint64_t us_span = us_last - us_first;
            uint64_t ideal   = us_span * (uint64_t)s_cycles_per_us;
            uint32_t actual_cyc = cyc_at_edge[TIMER_CAL_SAMPLES - 1u]
                                  - cyc_at_edge[0u];

            uint64_t thresh = ideal / 10u;
            bool span_ok    = ((uint64_t)actual_cyc >= ideal - thresh) &&
                              ((uint64_t)actual_cyc <= ideal + thresh);

            if (span_ok && ideal > 0u) {
                s_drift_ppm = (int32_t)(
                    ((int64_t)(uint64_t)actual_cyc - (int64_t)ideal)
                    * 1000000LL / (int64_t)ideal);
            } else {
                s_calibrating = false;
                s_dwt_phase   = 0u;
                s_drift_ppm   = 0;
                return;
            }
        }
    } else {
        s_drift_ppm = 0;
    }

    s_calibrated = true;

    if (s_dwt_ok && !s_watchdog_started) {
        UBaseType_t wdog_prio = (UBaseType_t)(configMAX_PRIORITIES - 2u);
        if (wdog_prio < (tskIDLE_PRIORITY + 1u))
            wdog_prio = tskIDLE_PRIORITY + 1u;

        BaseType_t rc = xTaskCreate(dwt_watchdog_task, "dwt_wdog",
                                    256u, NULL, wdog_prio, NULL);
        if (rc == pdPASS) {
            s_watchdog_started = true;
        } else {
            s_dwt_ok      = false;
            s_dwt_phase   = 0u;
            s_calibrated  = false;
            s_drift_ppm   = 0;
        }
    }

    s_calibrating = false;
}

/* =========================================================================
 * Wrap-watchdog verification
 * ========================================================================= */
bool timer_wrap_test(void)
{
    if (!s_calibrated || !s_dwt_ok) return false;

    uint32_t hi_snapshot;
    taskENTER_CRITICAL();
    hi_snapshot = s_dwt_hi;
    s_dwt_prev  = 0xFFFFFFFFu;
    taskEXIT_CRITICAL();

    uint32_t waited_ms = 0u;
    bool fired = false;
    while (waited_ms < TIMER_WRAP_WATCHDOG_MS * 2u) {
        vTaskDelay(pdMS_TO_TICKS(50u));
        waited_ms += 50u;
        if (s_dwt_hi != hi_snapshot) { fired = true; break; }
    }

    if (fired) {
        taskENTER_CRITICAL();
        s_dwt_hi   = hi_snapshot;
        s_dwt_prev = DWT_NOW();
        taskEXIT_CRITICAL();
    }

    return fired;
}

/* =========================================================================
 * Fine timestamp — ISR-safe, bounded retries
 * ========================================================================= */
uint64_t timer_get_time_ns(void)
{
    if (!s_ready) return 0ULL;
    return gtimer_us_64() * 1000ULL;
}

uint64_t timer_get_time_ns_fine(void)
{
    if (!s_ready) return 0ULL;

    if (s_calibrated && s_dwt_ok) {
        uint32_t u0, u1;
        uint64_t us;
        uint8_t  retries = 4u;
        do {
            u0 = DWT_NOW();
            us = gtimer_us_64();
            u1 = DWT_NOW();
        } while ((u1 - u0) >= s_cycles_per_us && --retries);

        if ((u1 - u0) >= s_cycles_per_us) {
            return us * 1000ULL;
        }

        uint32_t umid    = u0 + ((u1 - u0) >> 1u);
        uint32_t sub_cyc = (umid - s_dwt_phase + s_cycles_per_us)
                           % s_cycles_per_us;
        uint32_t sub_ns  = (sub_cyc * 1000u) / s_cycles_per_us;
        return us * 1000ULL + (uint64_t)sub_ns;
    }

    if (s_calibrated && s_systick_ok) {
        uint32_t u0, u1;
        uint64_t us;
        uint8_t  retries = 4u;
        do {
            u0 = SYSTICK_UP();
            us = gtimer_us_64();
            u1 = SYSTICK_UP();
        } while ((u1 - u0) >= s_cycles_per_us && --retries);

        if ((u1 - u0) >= s_cycles_per_us) {
            return us * 1000ULL;
        }

        uint32_t umid    = u0 + ((u1 - u0) >> 1u);
        uint32_t sub_cyc = (umid - s_systick_phase + s_cycles_per_us)
                           % s_cycles_per_us;
        uint32_t sub_ns  = (sub_cyc * 1000u) / s_cycles_per_us;
        return us * 1000ULL + (uint64_t)sub_ns;
    }

    return gtimer_us_64() * 1000ULL;
}

uint64_t timer_get_time_ns_isr(void) { return timer_get_time_ns_fine(); }

uint64_t timer_get_time_us(void)
{
    if (!s_ready) return 0ULL;
    return gtimer_us_64();
}

uint64_t timer_align_ns(uint64_t period_ns)
{
    if (period_ns == 0u) return timer_get_time_ns_fine();
    uint64_t now = timer_get_time_ns_fine();
    uint64_t rem = now % period_ns;
    return (rem == 0u) ? now : (now + period_ns - rem);
}

/* =========================================================================
 * 64-bit DWT — lock-free double-read retry
 * ========================================================================= */
uint32_t timer_get_cycles32(void) { return s_dwt_ok ? DWT_NOW() : 0u; }

uint64_t timer_get_cycles64(void)
{
    if (!s_dwt_ok) return 0ULL;
    uint32_t lo, hi1, hi2;
    do { hi1 = s_dwt_hi; lo = DWT_NOW(); hi2 = s_dwt_hi; } while (hi1 != hi2);
    return ((uint64_t)hi1 << 32) | lo;
}

/* =========================================================================
 * Unit conversion
 * ========================================================================= */
uint64_t timer_ns_to_cycles(uint64_t ns)
{
    if (s_cycles_per_us == 0u) return 0ULL;
    uint64_t us  = ns / 1000ULL;
    uint64_t sub = ns % 1000ULL;
    return us * s_cycles_per_us + (sub * s_cycles_per_us + 500ULL) / 1000ULL;
}

uint64_t timer_cycles_to_ns(uint64_t cycles)
{
    if (s_cycles_per_us == 0u) return 0ULL;
    uint64_t us  = cycles / s_cycles_per_us;
    uint64_t rem = cycles % s_cycles_per_us;
    return us * 1000ULL + (rem * 1000ULL) / s_cycles_per_us;
}

/* =========================================================================
 * Precision spin-wait — wrap-safe signed subtraction
 * ========================================================================= */
void timer_spin_until_ns(uint64_t target_abs_ns)
{
    if (!s_ready) return;
    uint64_t now = timer_get_time_ns_fine();
    if (now >= target_abs_ns) return;
    uint64_t delta_ns = target_abs_ns - now;
    if (delta_ns < TIMER_SPIN_MIN_NS || delta_ns > TIMER_SPIN_MAX_NS) return;

    if (s_calibrated && s_dwt_ok) {
        uint32_t cyc      = (uint32_t)timer_ns_to_cycles(delta_ns);
        uint32_t base     = DWT_NOW();
        uint32_t deadline = base + cyc;
        while ((int32_t)(deadline - DWT_NOW()) > 0)
            __asm__ volatile("isb" ::: "memory");
        return;
    }

    while (gtimer_us_64() * 1000ULL < target_abs_ns)
        __asm__ volatile("isb" ::: "memory");
}

void timer_spin_for_ns(uint64_t duration_ns)
{
    if (duration_ns < TIMER_SPIN_MIN_NS || duration_ns > TIMER_SPIN_MAX_NS) return;
    timer_spin_until_ns(timer_get_time_ns_fine() + duration_ns);
}

/* =========================================================================
 * Diagnostics
 * ========================================================================= */
bool     timer_is_ready(void)         { return s_ready; }
bool     timer_is_calibrated(void)    { return s_calibrated; }
uint32_t timer_get_cpu_freq_hz(void)  { return s_cpu_freq_hz; }
uint32_t timer_get_dwt_phase(void)    { return s_dwt_phase; }
uint32_t timer_get_wrap_count(void)   { return s_dwt_hi; }
int32_t  timer_get_drift_ppm(void)    { return s_drift_ppm; }
uint32_t timer_get_overrun_count(void){ return s_overrun_count; }

uint32_t timer_get_resolution_ns(void)
{
    if (s_calibrated && (s_dwt_ok || s_systick_ok))
        return s_fine_resolution_ns;
    return (uint32_t)TIMER_COARSE_RESOLUTION_NS;
}

const char *timer_get_resolution_source(void)
{
    if (s_calibrated && s_dwt_ok)     return "dwt";
    if (s_calibrated && s_systick_ok) return "systick";
    return "gtimer";
}
