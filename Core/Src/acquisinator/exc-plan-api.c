#include "exc-plan-api.h"

#include <math.h>

#include "sync-demod.h"

bool exc_plan_api_make(const struct ExcPlanConfig *cfg, uint32_t freq_hz, struct ExcPlan *plan) {
    // bound the frequency first, so freq * samples cannot wrap
    if (freq_hz == 0U || cfg->min_samples == 0U || freq_hz > cfg->max_tick_hz / cfg->min_samples) {
        return false;
    }
    uint32_t samples = cfg->max_tick_hz / freq_hz;
    if (samples > cfg->max_samples) {
        samples = cfg->max_samples;
    }

    // rounded timer period; one count longer if rounding pushed the tick past the budget
    uint32_t tick_hz = freq_hz * samples;
    uint32_t reload = (cfg->tim_clk_hz + tick_hz / 2U) / tick_hz;
    if ((uint64_t)reload * cfg->max_tick_hz < cfg->tim_clk_hz) {
        reload++;
    }
    if (reload == 0U) {
        return false;
    }
    float freq_real = (float)cfg->tim_clk_hz / ((float)reload * (float)samples);

    // next to a mains harmonic the demodulator takes its drift for the signal
    float harmonic = roundf(freq_real / cfg->mains_hz);
    if (harmonic >= 1.0F && fabsf(freq_real - harmonic * cfg->mains_hz) < cfg->mains_guard_hz) {
        return false;
    }

    // shortest window of whole periods that is also whole mains periods, within tolerance
    for (uint32_t p = 1U; p <= cfg->max_periods; p++) {
        uint64_t window_ticks = (uint64_t)p * samples * reload;
        float window_s = (float)window_ticks / (float)cfg->tim_clk_hz;
        if (window_s * 1000.0F < (float)cfg->min_window_ms) {
            continue;
        }
        if (window_s * 1000.0F > (float)cfg->max_window_ms) {
            break;
        }
        float mains_periods = roundf(cfg->mains_hz * window_s);
        if (mains_periods < 1.0F) {
            continue;
        }
        float mains_err = fabsf(mains_periods / window_s - cfg->mains_hz);
        if (mains_err > cfg->mains_tol_hz) {
            continue;
        }
        uint32_t output_every = (uint32_t)lroundf(freq_real * (float)cfg->output_ms / 1000.0F);
        plan->samples = samples;
        plan->autoreload = reload - 1U;
        plan->periods = p;
        plan->output_every = output_every == 0U ? 1U : output_every;
        plan->freq_hz = freq_real;
        plan->window_s = window_s;
        plan->mains_err_hz = mains_err;
        return true;
    }
    return false;
}

bool exc_plan_api_levels_valid(float dc_v, float amp_v, float out_min_v, float out_max_v) {
    const float half_mv = 0.0005F; // levels come in whole mV: the limits themselves are allowed
    return amp_v > 0.0F && dc_v - amp_v >= out_min_v - half_mv && dc_v + amp_v <= out_max_v + half_mv;
}

void exc_plan_api_make_dac_table(uint16_t *dac, const int16_t *sin_q15, uint32_t samples, float dc_v, float amp_v,
                                 float vdda_v, uint16_t dac_max) {
    for (uint32_t k = 0; k < samples; k++) {
        float v = dc_v + amp_v * (float)sin_q15[k] / (float)SYNC_DEMOD_Q15_ONE;
        long code = lroundf(v / vdda_v * (float)dac_max);
        if (code < 0) {
            code = 0;
        }
        if (code > (long)dac_max) {
            code = (long)dac_max;
        }
        dac[k] = (uint16_t)code;
    }
}
