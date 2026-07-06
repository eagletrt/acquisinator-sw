#include "lpf-api.h"
#include "math.h"

static const float TWO_PI = 2.0F * (float)M_PI;

void lpf_api_init(struct LPFHandler *hlpf, float cutoff_freq, float sampling_freq) {
    float sample_period = 1.0F / sampling_freq;             // Sampling period
    float time_const = 1.0F / (TWO_PI * cutoff_freq);       // RC time constant
    hlpf->alpha = sample_period / (time_const + sample_period);
    hlpf->prev_output = 0.0F;
}

float lpf_api_update(struct LPFHandler *hlpf, float input) {
    hlpf->prev_output = hlpf->alpha * input + (1.0F - hlpf->alpha) * hlpf->prev_output;
    return hlpf->prev_output;
}

void lpf_api_reset(struct LPFHandler *hlpf) {
    hlpf->prev_output = 0.0F;
}
