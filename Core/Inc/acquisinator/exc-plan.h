#ifndef EXC_PLAN_H
#define EXC_PLAN_H

#include <stdint.h>

/*
 * Limits the excitation has to respect. The lock-in itself works at any
 * frequency (every sample is tagged with the phase index of the DAC value that
 * produced it), so these only protect the ISR budget and the mains rejection.
 */
struct ExcPlanConfig {
    uint32_t tim_clk_hz;    // Timer clock driving the DAC/ADC tick
    uint32_t max_samples;   // Samples per period when the tick allows it
    uint32_t min_samples;   // Fewest samples per period accepted
    uint32_t max_tick_hz;   // Fastest tick the ISR keeps up with
    uint32_t max_periods;   // Longest window the demodulator holds, in periods
    uint32_t min_window_ms; // Shortest window
    uint32_t max_window_ms; // Longest window
    uint32_t output_ms;     // Target time between two results (sliding window)
    float mains_hz;         // Mains frequency to reject
    float mains_tol_hz;     // Allowed error of the window on whole mains periods, in Hz
    float mains_guard_hz;   // Least distance of the excitation from any mains harmonic
};

/*
 * Timing for one excitation frequency. The window lasts `periods` whole
 * excitation periods and, within mains_tol_hz, whole mains periods: 50 Hz and
 * its harmonics cancel. When the mains is off by d Hz (the grid drifts by
 * ~0.1 Hz) harmonic k leaks into the result by about
 *     k * (d + mains_err_hz) / |freq - k * mains_hz|   (relative to its amplitude)
 * so the excitation is kept at least mains_guard_hz away from every harmonic.
 */
struct ExcPlan {
    uint32_t samples;      // Samples per excitation period (table length)
    uint32_t autoreload;   // Timer auto-reload: tick = tim_clk / (autoreload + 1)
    uint32_t periods;      // Excitation periods per window
    uint32_t output_every; // Periods between two results
    float freq_hz;         // Frequency actually generated (autoreload is rounded)
    float window_s;        // Window length
    float mains_err_hz;    // Window error on whole mains periods, as a mains frequency error
};

#endif /* EXC_PLAN_H */
