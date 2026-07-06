#ifndef SINE_LUT_API_H
#define SINE_LUT_API_H

#include "sine-lut.h"

// Fill `buffer` (of `size` samples) with one full period of a sine centered at
// mid-scale and rewind the read index:
//     buffer[i] = round((full_scale / 2) * (1 + sin(2*pi*i/size)))
// so samples span [0, full_scale] with the midpoint at full_scale/2.
void sine_lut_api_init(struct SineLUT *lut, uint16_t *buffer, uint32_t size, uint16_t full_scale);

// Return the sample at the current index and advance it, wrapping at the end.
uint16_t sine_lut_api_next(struct SineLUT *lut);

// Rewind the read index to the start of the period.
void sine_lut_api_reset(struct SineLUT *lut);

#endif /* SINE_LUT_API_H */
