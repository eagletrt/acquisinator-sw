#ifndef LOCKIN_API_H
#define LOCKIN_API_H

#include "lockin.h"

void lockin_api_init(struct LockInHandler *hlockin, float ref_freq, float lpf_cutoff, float sampling_freq, float ref_amplitude);
float lockin_api_update(struct LockInHandler *hlockin, float signal, float reference);
void lockin_api_reset(struct LockInHandler *hlockin);

#endif /* LOCKIN_API_H */