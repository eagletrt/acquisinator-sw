#ifndef SYNC_DEMOD_H
#define SYNC_DEMOD_H

#include <stdint.h>

/* Full scale of the Q15 reference tables (sin/cos scaled to +-SYNC_DEMOD_Q15_ONE). */
#define SYNC_DEMOD_Q15_ONE (32767)

/* Longest window, in excitation periods (sizes the per-period ring buffer). */
#define SYNC_DEMOD_MAX_PERIODS (25U)

/* Sums of one excitation period. */
struct SyncDemodPeriod {
    int64_t acc_i;
    int64_t acc_q;
    uint64_t acc_dc;
    uint16_t min;
    uint16_t max;
};

/*
 * Synchronous (block-averaging) lock-in demodulator.
 *
 * Every sample is tagged with the phase index of the excitation it was taken
 * at, so the reference is read straight from the same tables that drive the
 * DAC: no phase-shifting filter and no DC in the reference. Products are
 * accumulated over a window of an integer number of periods, which rejects
 * the input DC and the 2f term exactly (no ripple, unlike an IIR low-pass).
 *
 * The sums are kept per period in a ring buffer, so the window can slide: a
 * result is produced every `output_every` periods, always over the last
 * `periods_per_window` whole periods. With output_every == periods_per_window
 * (the default) the windows do not overlap, as in a plain block average.
 *
 * For an input  x[k] = dc + A*sin(2*pi*k/period + phi)  a completed window gives
 *     i = A*cos(phi),  q = A*sin(phi),  dc = mean(x)
 * all expressed in input units (e.g. ADC counts).
 */
struct SyncDemodHandler {
    const int16_t *sin_q15; // Q15 sin table, one period (caller-owned)
    const int16_t *cos_q15; // Q15 cos table, one period (caller-owned)
    uint32_t period;        // Samples per excitation period
    uint32_t window;        // Samples per window (period * periods_per_window)
    uint32_t periods_per_window;
    uint32_t output_every;  // Periods between two results

    // Running period
    int64_t acc_i;
    int64_t acc_q;
    uint64_t acc_dc;
    uint32_t count;
    uint16_t run_min;
    uint16_t run_max;

    // Completed periods of the current window
    struct SyncDemodPeriod ring[SYNC_DEMOD_MAX_PERIODS];
    uint32_t ring_head;    // Next slot to write
    uint32_t ring_filled;  // Valid periods in the ring (up to periods_per_window)
    uint32_t since_output; // Periods completed since the last result

    // Last completed window
    float i;          // In-phase amplitude (peak, input units)
    float q;          // Quadrature amplitude (peak, input units)
    float dc;         // Mean of the input over the window
    uint16_t min;     // Smallest input sample in the window
    uint16_t max;     // Largest input sample in the window
};

#endif /* SYNC_DEMOD_H */
