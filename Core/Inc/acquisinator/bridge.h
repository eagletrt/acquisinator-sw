#ifndef BRIDGE_H
#define BRIDGE_H

#include <stdint.h>

/* Flags of ACQUISINATOR_BRIDGE (bit 3 unused: it was "uncalibrated"). */
#define BRIDGE_FLAG_CLIP_LOW (1U << 0)
#define BRIDGE_FLAG_CLIP_HIGH (1U << 1)
#define BRIDGE_FLAG_SPI_ERROR (1U << 2)
#define BRIDGE_FLAG_NO_DATA (1U << 4)
#define BRIDGE_FLAG_OVERRUN (1U << 5)

/* Analog chain from the bridge to the ADC. */
struct BridgeConfig {
    float amp_gain;      // Differential gain of the amplifier
    float adc_lsb_v;     // ADC volts per count
    float cal_phase_rad; // Phase of the analog chain, projected out of the in-phase component
    float clip_low_v;    // Amplifier output saturation thresholds
    float clip_high_v;
};

/*
 * One window converted. The bridge output is ratiometric: in-phase amplitude
 * at the bridge over excitation amplitude, in mV/V like a load cell. For a
 * quarter bridge it is about GF * strain / 4, and force is linear in it: a
 * force calibration needs only this value.
 */
struct BridgeReading {
    float x_v;          // In-phase amplitude at the amplifier output, signed [V]
    float phase_deg;    // Phase of the amplifier output against the excitation
    float out_mv_per_v; // Bridge output, signed [mV/V]
    uint8_t flags;
};

#endif /* BRIDGE_H */
