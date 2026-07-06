#ifndef LPF_API_H
#define LPF_API_H

#include "lpf.h"

void lpf_api_init(struct LPFHandler *hlpf, float cutoff_freq, float sampling_freq);

float lpf_api_update(struct LPFHandler *hlpf, float input);

void lpf_api_reset(struct LPFHandler *hlpf);

#endif /* LPF_API_H */