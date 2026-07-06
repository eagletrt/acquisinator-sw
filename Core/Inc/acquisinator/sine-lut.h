#ifndef SINE_LUT_H
#define SINE_LUT_H

#include <stdint.h>

/*
 * Sine lookup table used to synthesize the excitation reference on the DAC.
 * The sample buffer is owned by the caller and filled once at init; the handle
 * only tracks its geometry and the current read position.
 */
struct SineLUT {
    uint16_t *samples; // Caller-owned sample buffer (one full period)
    uint32_t size;     // Number of samples per period
    uint32_t index;    // Current read position
};

#endif /* SINE_LUT_H */
