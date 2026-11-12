#ifndef EXC_RUNNER_H
#define EXC_RUNNER_H

#include <stdbool.h>
#include <stdint.h>

#include "exc-plan.h"
#include "sync-demod.h"

/* Longest excitation table (samples per period). */
#define EXC_RUNNER_MAX_SAMPLES (64U)

/* Limits and defaults of the excitation. */
struct ExcRunnerConfig {
    struct ExcPlanConfig plan;
    float vdda_v;        // DAC full scale
    float out_min_v;     // DAC output range (buffer enabled)
    float out_max_v;
    uint16_t dac_max;    // DAC full-scale code
    uint16_t default_freq_hz;
    uint16_t default_dc_mv;
    uint16_t default_amp_mv;
};

/* One excitation setting: timing plus the tables it runs on. */
struct ExcSetting {
    struct ExcPlan plan;
    uint16_t freq_req_hz; // As requested (plan.freq_hz is the one generated)
    uint16_t dc_mv;
    uint16_t amp_mv;
    uint8_t id;           // +1 at every applied setting
    int16_t sin_q15[EXC_RUNNER_MAX_SAMPLES];
    int16_t cos_q15[EXC_RUNNER_MAX_SAMPLES];
    uint16_t dac[EXC_RUNNER_MAX_SAMPLES];
};

/* Result of one completed lock-in window. */
struct LockinResult {
    float i;          // In-phase amplitude [ADC counts, peak]
    float q;          // Quadrature amplitude [ADC counts, peak]
    float dc;         // Mean amplifier output [ADC counts]
    uint16_t min;     // Smallest sample in the window [ADC counts]
    uint16_t max;     // Largest sample in the window [ADC counts]
    float amp_v;      // Excitation amplitude the window was taken at
    uint8_t exc_id;   // Excitation setting the window was taken at
    bool spi_error;   // A sample was lost since the previous result
    bool overrun;     // The ISR outlasted a tick during the window
};

/* What the ISR has to do after advancing one tick. */
struct ExcStep {
    uint16_t dac;        // DAC code to write now
    uint32_t phase;      // Phase index of the sample just taken
    bool switched;       // A new setting was adopted at this tick
    uint32_t autoreload; // Timer auto-reload to write when switched
};

/*
 * Excitation and acquisition, one call pair per timer tick:
 *   exc_runner_api_advance  right after the ADC sample, then write step.dac;
 *   exc_runner_api_sample   with that sample.
 * The sample taken at a tick is the amplifier output settled on the DAC value
 * written at the previous tick, so it is demodulated with that value's phase.
 *
 * Two settings: the ISR runs on `active` while the main loop prepares the
 * other one, which is adopted at the end of a period (exc_runner_api_request).
 */
struct ExcRunner {
    const struct ExcRunnerConfig *cfg;
    struct ExcSetting slots[2];
    struct ExcSetting *volatile active;
    struct ExcSetting *volatile pending; // Ready, adopted at the end of the period
    struct SyncDemodHandler demod;
    uint32_t idx;          // Phase index of the DAC value being output
    bool first_window;     // The next result is dropped (start-up or new setting)
    bool window_spi_error;
    bool window_overrun;
};

#endif /* EXC_RUNNER_H */
