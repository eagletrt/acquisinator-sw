#include "demod.h"

#define MY_PI   (3.14f)
#define MY_PI_4 (0.78f)
#define MY_PI_2 (1.57f)

static float my_sqrtf(float x)
{
    if (x <= 0.0f) return 0.0f;
    union { float f; unsigned int u; } c = { .f = x };
    c.u  = 0x5f3759dfU - (c.u >> 1);
    float y = c.f;
    y *= 1.5f - 0.5f * x * y * y;
    y *= 1.5f - 0.5f * x * y * y;
    y *= 1.5f - 0.5f * x * y * y;
    return x * y;
}

static float my_tanf(float x)
{
    int recip = 0;
    if (x > MY_PI_4) { x = MY_PI_2 - x; recip = 1; }
    float x2 = x * x;
    float t = x * (1.0f + x2 * (0.33333333f + x2 * (0.13333333f +
              x2 * (0.05396825f + x2 * (0.02186947f + x2 * 0.00886257f)))));
    return recip ? 1.0f / t : t;
}

static float my_atan2f(float y, float x)
{
    float abs_y = (y < 0.0f) ? -y : y;
    float r, angle;
    if (x >= 0.0f) {
        float denom = x + abs_y;
        if (denom == 0.0f) return 0.0f;
        r     = (x - abs_y) / denom;
        angle = MY_PI_4 + (0.1963f * r * r - 0.9817f) * r;
    } else {
        r     = (x + abs_y) / (abs_y - x);
        angle = 3.0f * MY_PI_4 + (0.1963f * r * r - 0.9817f) * r;
    }
    return (y < 0.0f) ? -angle : angle;
}

void Demod_Init(Demod_HandleTypeDef *hdemod, float ref_freq, float lpf_cutoff, float sampling_freq)
{
    float t = my_tanf(MY_PI * ref_freq / sampling_freq);
    hdemod->allpass_coeff  = (1.0f - t) / (1.0f + t);
    hdemod->allpass_x_prev = 0.0f;
    hdemod->allpass_y_prev = 0.0f;

    LPF_Init(&hdemod->lpf_i, lpf_cutoff, sampling_freq);
    LPF_Init(&hdemod->lpf_q, lpf_cutoff, sampling_freq);

    hdemod->i         = 0.0f;
    hdemod->q         = 0.0f;
    hdemod->amplitude = 0.0f;
    hdemod->phase     = 0.0f;
}

void Demod_Update(Demod_HandleTypeDef *hdemod, float signal, float reference)
{
    float ref_q = hdemod->allpass_coeff * reference
                + hdemod->allpass_x_prev
                - hdemod->allpass_coeff * hdemod->allpass_y_prev;

    hdemod->allpass_x_prev = reference;
    hdemod->allpass_y_prev = ref_q;

    hdemod->i = LPF_Update(&hdemod->lpf_i, signal * reference);
    hdemod->q = LPF_Update(&hdemod->lpf_q, signal * ref_q);

    hdemod->amplitude = my_sqrtf(hdemod->i * hdemod->i + hdemod->q * hdemod->q);
    hdemod->phase     = my_atan2f(hdemod->q, hdemod->i);
}

void Demod_Reset(Demod_HandleTypeDef *hdemod)
{
    hdemod->allpass_x_prev = 0.0f;
    hdemod->allpass_y_prev = 0.0f;

    LPF_Reset(&hdemod->lpf_i);
    LPF_Reset(&hdemod->lpf_q);

    hdemod->i         = 0.0f;
    hdemod->q         = 0.0f;
    hdemod->amplitude = 0.0f;
    hdemod->phase     = 0.0f;
}
