#include "sync-demod-api.h"

#include <math.h>

void sync_demod_api_make_tables(int16_t *sin_q15, int16_t *cos_q15, uint32_t period) {
    for (uint32_t k = 0; k < period; k++) {
        float theta = 2.0F * (float)M_PI * (float)k / (float)period;
        sin_q15[k] = (int16_t)lroundf((float)SYNC_DEMOD_Q15_ONE * sinf(theta));
        cos_q15[k] = (int16_t)lroundf((float)SYNC_DEMOD_Q15_ONE * cosf(theta));
    }
}

void sync_demod_api_init(struct SyncDemodHandler *hdemod, const int16_t *sin_q15, const int16_t *cos_q15, uint32_t period, uint32_t periods_per_window) {
    if (periods_per_window > SYNC_DEMOD_MAX_PERIODS) {
        periods_per_window = SYNC_DEMOD_MAX_PERIODS;
    }
    if (periods_per_window == 0U) {
        periods_per_window = 1U;
    }
    hdemod->sin_q15 = sin_q15;
    hdemod->cos_q15 = cos_q15;
    hdemod->period = period;
    hdemod->periods_per_window = periods_per_window;
    hdemod->window = period * periods_per_window;
    hdemod->output_every = periods_per_window;

    hdemod->i = 0.0F;
    hdemod->q = 0.0F;
    hdemod->dc = 0.0F;
    hdemod->min = 0;
    hdemod->max = 0;

    sync_demod_api_reset(hdemod);
}

void sync_demod_api_set_output_every(struct SyncDemodHandler *hdemod, uint32_t periods) {
    hdemod->output_every = periods == 0U ? 1U : periods;
    hdemod->since_output = 0;
}

static void period_reset(struct SyncDemodHandler *hdemod) {
    hdemod->acc_i = 0;
    hdemod->acc_q = 0;
    hdemod->acc_dc = 0;
    hdemod->count = 0;
    hdemod->run_min = UINT16_MAX;
    hdemod->run_max = 0;
}

bool sync_demod_api_update(struct SyncDemodHandler *hdemod, uint16_t sample, uint32_t phase) {
    hdemod->acc_i += (int64_t)sample * hdemod->sin_q15[phase];
    hdemod->acc_q += (int64_t)sample * hdemod->cos_q15[phase];
    hdemod->acc_dc += sample;

    if (sample < hdemod->run_min) {
        hdemod->run_min = sample;
    }
    if (sample > hdemod->run_max) {
        hdemod->run_max = sample;
    }

    hdemod->count++;
    if (hdemod->count < hdemod->period) {
        return false;
    }

    // a whole period: into the ring, overwriting the oldest one
    struct SyncDemodPeriod *slot = &hdemod->ring[hdemod->ring_head];
    slot->acc_i = hdemod->acc_i;
    slot->acc_q = hdemod->acc_q;
    slot->acc_dc = hdemod->acc_dc;
    slot->min = hdemod->run_min;
    slot->max = hdemod->run_max;
    hdemod->ring_head = (hdemod->ring_head + 1U) % hdemod->periods_per_window;
    if (hdemod->ring_filled < hdemod->periods_per_window) {
        hdemod->ring_filled++;
    }
    hdemod->since_output++;
    period_reset(hdemod);

    if (hdemod->ring_filled < hdemod->periods_per_window || hdemod->since_output < hdemod->output_every) {
        return false;
    }
    hdemod->since_output = 0;

    int64_t acc_i = 0;
    int64_t acc_q = 0;
    uint64_t acc_dc = 0;
    uint16_t min = UINT16_MAX;
    uint16_t max = 0;
    for (uint32_t p = 0; p < hdemod->periods_per_window; p++) {
        const struct SyncDemodPeriod *s = &hdemod->ring[p];
        acc_i += s->acc_i;
        acc_q += s->acc_q;
        acc_dc += s->acc_dc;
        min = s->min < min ? s->min : min;
        max = s->max > max ? s->max : max;
    }

    // sum(x*sin) over whole periods = A*cos(phi)*N/2 (times the Q15 scale)
    float scale = 2.0F / ((float)hdemod->window * (float)SYNC_DEMOD_Q15_ONE);
    hdemod->i = (float)acc_i * scale;
    hdemod->q = (float)acc_q * scale;
    hdemod->dc = (float)acc_dc / (float)hdemod->window;
    hdemod->min = min;
    hdemod->max = max;
    return true;
}

void sync_demod_api_reset(struct SyncDemodHandler *hdemod) {
    period_reset(hdemod);
    hdemod->ring_head = 0;
    hdemod->ring_filled = 0;
    hdemod->since_output = 0;
}
