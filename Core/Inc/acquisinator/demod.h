#ifndef DEMOD_H
#define DEMOD_H

#include "lpf.h"

struct DemodHandler {
    float allpass_coeff;
    float allpass_x_prev;
    float allpass_y_prev;
    struct LPFHandler lpf_i;
    struct LPFHandler lpf_q;
    float i;
    float q;
    float amplitude;
    float phase;
};

#endif /* DEMOD_H */
