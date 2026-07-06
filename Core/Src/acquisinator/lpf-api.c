#include "lpf-api.h"
#include "math.h"

void lpf_api_init(struct LPFHandler *hlpf, float cutoff_freq, float sampling_freq) {
    float dt = 1.0f / sampling_freq;               // Sampling period
    float rc = 1.0f / (2.0f * M_PI * cutoff_freq); // RC time constant
    hlpf->alpha = dt / (rc + dt);
    hlpf->prev_output = 0.0f;
}

float lpf_api_update(struct LPFHandler *hlpf, float input) {
    hlpf->prev_output = hlpf->alpha * input + (1.0f - hlpf->alpha) * hlpf->prev_output;
    return hlpf->prev_output;
}

void lpf_api_reset(struct LPFHandler *hlpf) {
    hlpf->prev_output = 0.0f;
}
