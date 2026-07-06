#ifndef LOCKIN_H
#define LOCKIN_H

#include "demod.h"

/* Regularization offset added to A^2 to keep the transfer output well-behaved
 * when the reference amplitude is small (from the validated bench test). */
#define LOCKIN_OFFSET (0.5f)

/*
 * Lock-in amplifier API.
 *
 * Wraps the quadrature demodulator (demod) and closes the chain by turning the
 * recovered amplitude into the measured transfer output:
 *
 *     output = amplitude / (ref_amplitude^2 + LOCKIN_OFFSET)
 *
 * where ref_amplitude is the amplitude of the excitation reference (e.g. VCC/2).
 */
struct LockInHandler {
    struct DemodHandler demod;
    float ref_amplitude; // Amplitude A of the excitation reference
    float amplitude;     // Last recovered amplitude (sqrt(I^2 + Q^2))
    float phase;         // Last recovered phase (atan2(Q, I))
    float output;        // Last computed transfer output
};

#endif /* LOCKIN_H */
