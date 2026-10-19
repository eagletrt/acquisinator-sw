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
    hdemod->sin_q15 = sin_q15;
    hdemod->cos_q15 = cos_q15;
    hdemod->period = period;
    hdemod->window = period * periods_per_window;

    hdemod->i = 0.0F;
    hdemod->q = 0.0F;
    hdemod->dc = 0.0F;
    hdemod->min = 0;
    hdemod->max = 0;

    sync_demod_api_reset(hdemod);
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
    if (hdemod->count < hdemod->window) {
        return false;
    }

    // sum(x*sin) over whole periods = A*cos(phi)*N/2 (times the Q15 scale)
    float scale = 2.0F / ((float)hdemod->window * (float)SYNC_DEMOD_Q15_ONE);
    hdemod->i = (float)hdemod->acc_i * scale;
    hdemod->q = (float)hdemod->acc_q * scale;
    hdemod->dc = (float)hdemod->acc_dc / (float)hdemod->window;
    hdemod->min = hdemod->run_min;
    hdemod->max = hdemod->run_max;

    sync_demod_api_reset(hdemod);
    return true;
}

void sync_demod_api_reset(struct SyncDemodHandler *hdemod) {
    hdemod->acc_i = 0;
    hdemod->acc_q = 0;
    hdemod->acc_dc = 0;
    hdemod->count = 0;
    hdemod->run_min = UINT16_MAX;
    hdemod->run_max = 0;
}
