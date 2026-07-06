#include "demod-api.h"
#include "demod.h"
#include "lpf-api.h"

#include <math.h>

void demod_api_init(struct DemodHandler *hdemod, float ref_freq, float lpf_cutoff, float sampling_freq) {
    float tan_val = tanf((float)M_PI * ref_freq / sampling_freq);
    hdemod->allpass_coeff = (1.0F - tan_val) / (1.0F + tan_val);
    hdemod->allpass_x_prev = 0.0F;
    hdemod->allpass_y_prev = 0.0F;

    lpf_api_init(&hdemod->lpf_i, lpf_cutoff, sampling_freq);
    lpf_api_init(&hdemod->lpf_q, lpf_cutoff, sampling_freq);

    hdemod->i = 0.0F;
    hdemod->q = 0.0F;
    hdemod->amplitude = 0.0F;
    hdemod->phase = 0.0F;
}

void demod_api_update(struct DemodHandler *hdemod, float signal, float reference) {
    float ref_q = hdemod->allpass_coeff * reference + hdemod->allpass_x_prev - hdemod->allpass_coeff * hdemod->allpass_y_prev;

    hdemod->allpass_x_prev = reference;
    hdemod->allpass_y_prev = ref_q;

    hdemod->i = lpf_api_update(&hdemod->lpf_i, signal * reference);
    hdemod->q = lpf_api_update(&hdemod->lpf_q, signal * ref_q);

    hdemod->amplitude = sqrtf(hdemod->i * hdemod->i + hdemod->q * hdemod->q);
    hdemod->phase = atan2f(hdemod->q, hdemod->i);
}

void demod_api_reset(struct DemodHandler *hdemod) {
    hdemod->allpass_x_prev = 0.0F;
    hdemod->allpass_y_prev = 0.0F;

    lpf_api_reset(&hdemod->lpf_i);
    lpf_api_reset(&hdemod->lpf_q);

    hdemod->i = 0.0F;
    hdemod->q = 0.0F;
    hdemod->amplitude = 0.0F;
    hdemod->phase = 0.0F;
}
