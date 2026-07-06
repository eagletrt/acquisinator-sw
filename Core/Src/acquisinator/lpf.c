#include "lpf.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void LPF_Init(LPF_HandleTypeDef *hlpf, float cutoff_freq, float sampling_freq)
{
    float dt = 1.0f / sampling_freq; // Sampling period
    float rc = 1.0f / (2.0f * M_PI * cutoff_freq); // RC time constant
    hlpf->alpha = dt / (rc + dt);
    hlpf->prev_output = 0.0f;
}

float LPF_Update(LPF_HandleTypeDef *hlpf, float input)
{
    hlpf->prev_output = hlpf->alpha * input + (1.0f - hlpf->alpha) * hlpf->prev_output;
    return hlpf->prev_output;
}

void LPF_Reset(LPF_HandleTypeDef *hlpf)
{
    hlpf->prev_output = 0.0f;
}
