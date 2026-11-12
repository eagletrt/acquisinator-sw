#include "exc-runner-api.h"

#include <math.h>
#include <stdatomic.h>
#include <stddef.h>

#include "exc-plan-api.h"
#include "sync-demod-api.h"

#define MV_PER_V 1000.0F
#define STATUS_ID_BYTE 7U

static bool setting_make(const struct ExcRunnerConfig *cfg, struct ExcSetting *exc, uint32_t freq_hz, uint32_t dc_mv,
                         uint32_t amp_mv, uint8_t id) {
    float dc_v = (float)dc_mv / MV_PER_V;
    float amp_v = (float)amp_mv / MV_PER_V;
    if (freq_hz > UINT16_MAX || !exc_plan_api_levels_valid(dc_v, amp_v, cfg->out_min_v, cfg->out_max_v) ||
        !exc_plan_api_make(&cfg->plan, freq_hz, &exc->plan) || exc->plan.samples > EXC_RUNNER_MAX_SAMPLES) {
        return false;
    }
    sync_demod_api_make_tables(exc->sin_q15, exc->cos_q15, exc->plan.samples);
    exc_plan_api_make_dac_table(exc->dac, exc->sin_q15, exc->plan.samples, dc_v, amp_v, cfg->vdda_v, cfg->dac_max);
    exc->freq_req_hz = (uint16_t)freq_hz;
    exc->dc_mv = (uint16_t)dc_mv;
    exc->amp_mv = (uint16_t)amp_mv;
    exc->id = id;
    return true;
}

static void demod_init(struct ExcRunner *hrunner, const struct ExcSetting *exc) {
    sync_demod_api_init(&hrunner->demod, exc->sin_q15, exc->cos_q15, exc->plan.samples, exc->plan.periods);
    sync_demod_api_set_output_every(&hrunner->demod, exc->plan.output_every);
    hrunner->first_window = true;
    hrunner->window_spi_error = false;
    hrunner->window_overrun = false;
}

bool exc_runner_api_init(struct ExcRunner *hrunner, const struct ExcRunnerConfig *cfg) {
    hrunner->cfg = cfg;
    hrunner->pending = NULL;
    hrunner->idx = 0;
    if (!setting_make(cfg, &hrunner->slots[0], cfg->default_freq_hz, cfg->default_dc_mv, cfg->default_amp_mv, 0)) {
        return false;
    }
    hrunner->active = &hrunner->slots[0];
    demod_init(hrunner, hrunner->active);
    return true;
}

bool exc_runner_api_request(struct ExcRunner *hrunner, uint32_t freq_hz, uint32_t dc_mv, uint32_t amp_mv) {
    if (hrunner->pending != NULL) {
        return false;
    }
    const struct ExcSetting *cur = hrunner->active; // stable: the ISR only switches to `pending`
    struct ExcSetting *next = cur == &hrunner->slots[0] ? &hrunner->slots[1] : &hrunner->slots[0];
    if (!setting_make(hrunner->cfg, next, freq_hz != 0U ? freq_hz : cur->freq_req_hz, dc_mv != 0U ? dc_mv : cur->dc_mv,
                      amp_mv != 0U ? amp_mv : cur->amp_mv, (uint8_t)(cur->id + 1U))) {
        return false;
    }
    // tables written before the ISR can see them (same core: a compiler barrier is enough)
    atomic_signal_fence(memory_order_release);
    hrunner->pending = next;
    return true;
}

void exc_runner_api_advance(struct ExcRunner *hrunner, struct ExcStep *step) {
    const struct ExcSetting *exc = hrunner->active;
    step->phase = hrunner->idx;
    step->switched = false;
    hrunner->idx = (hrunner->idx + 1U) % exc->plan.samples;

    if (hrunner->idx == 0U && hrunner->pending != NULL) {
        // New setting at the end of a period: tables, timer period and window together.
        // With the timer preload the tick in progress keeps the old period: the first
        // window, dropped, also covers the settling of the analog chain.
        exc = hrunner->pending;
        hrunner->active = hrunner->pending;
        hrunner->pending = NULL;
        demod_init(hrunner, exc);
        step->switched = true;
        step->autoreload = exc->plan.autoreload;
    }
    step->dac = exc->dac[hrunner->idx];
}

bool exc_runner_api_sample(struct ExcRunner *hrunner, const struct ExcStep *step, bool sample_ok, uint16_t raw,
                           struct LockinResult *res) {
    if (step->switched) {
        return false; // taken with the old setting
    }
    if (!sample_ok) {
        // a missing sample would break the whole periods: the window starts again
        sync_demod_api_reset(&hrunner->demod);
        hrunner->window_spi_error = true;
        return false;
    }
    if (!sync_demod_api_update(&hrunner->demod, raw, step->phase)) {
        return false;
    }
    if (hrunner->first_window) {
        // first ADC sample after start-up, or settling after a new setting
        hrunner->first_window = false;
        hrunner->window_spi_error = false;
        hrunner->window_overrun = false;
        return false;
    }

    const struct ExcSetting *exc = hrunner->active;
    res->i = hrunner->demod.i;
    res->q = hrunner->demod.q;
    res->dc = hrunner->demod.dc;
    res->min = hrunner->demod.min;
    res->max = hrunner->demod.max;
    res->amp_v = (float)exc->amp_mv / MV_PER_V;
    res->exc_id = exc->id;
    res->spi_error = hrunner->window_spi_error;
    res->overrun = hrunner->window_overrun;
    hrunner->window_spi_error = false;
    hrunner->window_overrun = false;
    return true;
}

void exc_runner_api_mark_overrun(struct ExcRunner *hrunner) {
    hrunner->window_overrun = true;
}

void exc_runner_api_pack_status(const struct ExcSetting *exc, uint8_t data[8]) {
    uint16_t freq_dhz = (uint16_t)lroundf(exc->plan.freq_hz * 10.0F);
    data[0] = (uint8_t)freq_dhz;
    data[1] = (uint8_t)(freq_dhz >> 8);
    data[2] = (uint8_t)exc->plan.samples;
    data[3] = (uint8_t)exc->plan.periods;
    data[4] = (uint8_t)exc->amp_mv;
    data[5] = (uint8_t)(((exc->amp_mv >> 8) & 0x0FU) | ((exc->dc_mv & 0x0FU) << 4));
    data[6] = (uint8_t)(exc->dc_mv >> 4);
    data[STATUS_ID_BYTE] = exc->id;
}
