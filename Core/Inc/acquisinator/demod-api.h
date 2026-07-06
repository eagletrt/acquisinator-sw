#ifndef DEMOD_API_H
#define DEMOD_API_H

#include "demod.h"

void demod_api_init(struct DemodHandler *hdemod, float ref_freq, float lpf_cutoff, float sampling_freq);
void demod_api_update(struct DemodHandler *hdemod, float signal, float reference);
void demod_api_reset(struct DemodHandler *hdemod);

#endif /* DEMOD_API_H */