#include "lockin-api.h"
#include "demod-api.h"

void lockin_api_init(struct LockInHandler *hlockin, float ref_freq, float lpf_cutoff, float sampling_freq, float ref_amplitude) {
    demod_api_init(&hlockin->demod, ref_freq, lpf_cutoff, sampling_freq);

    hlockin->ref_amplitude = ref_amplitude;
    hlockin->amplitude = 0.0F;
    hlockin->phase = 0.0F;
    hlockin->output = 0.0F;
}

float lockin_api_update(struct LockInHandler *hlockin, float signal, float reference) {
    demod_api_update(&hlockin->demod, signal, reference);

    hlockin->amplitude = hlockin->demod.amplitude;
    hlockin->phase = hlockin->demod.phase;

    // H = Y / (A^2 + offset), where A is the reference amplitude
    float ref_amp = hlockin->ref_amplitude;
    hlockin->output = hlockin->amplitude / (ref_amp * ref_amp + LOCKIN_OFFSET);

    return hlockin->output;
}

void lockin_api_reset(struct LockInHandler *hlockin) {
    demod_api_reset(&hlockin->demod);

    hlockin->amplitude = 0.0F;
    hlockin->phase = 0.0F;
    hlockin->output = 0.0F;
}
