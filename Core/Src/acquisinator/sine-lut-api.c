#include "sine-lut-api.h"

#include <math.h>

void sine_lut_api_init(struct SineLUT *lut, uint16_t *buffer, uint32_t size, uint16_t full_scale) {
    lut->samples = buffer;
    lut->size = size;
    lut->index = 0;

    for (uint32_t i = 0; i < size; i++) {
        float s = sinf(2.0f * (float)M_PI * (float)i / (float)size);
        buffer[i] = (uint16_t)((full_scale / 2.0f) * (1.0f + s) + 0.5f);
    }
}

uint16_t sine_lut_api_next(struct SineLUT *lut) {
    uint16_t sample = lut->samples[lut->index];
    lut->index = (lut->index + 1) % lut->size;
    return sample;
}

void sine_lut_api_reset(struct SineLUT *lut) {
    lut->index = 0;
}
