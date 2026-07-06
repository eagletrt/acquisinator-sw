#ifndef DEMOD_H
#define DEMOD_H

#include "lpf.h"

typedef struct {
    float allpass_coeff;
    float allpass_x_prev;
    float allpass_y_prev;
    LPF_HandleTypeDef lpf_i;
    LPF_HandleTypeDef lpf_q;
    float i;
    float q;
    float amplitude;
    float phase;
} Demod_HandleTypeDef;

void  Demod_Init   (Demod_HandleTypeDef *hdemod, float ref_freq, float lpf_cutoff, float sampling_freq);
void  Demod_Update (Demod_HandleTypeDef *hdemod, float signal, float reference);
void  Demod_Reset  (Demod_HandleTypeDef *hdemod);

#endif /* DEMOD_H */
