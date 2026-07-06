#ifndef LPF_H
#define LPF_H

typedef struct {
    float alpha;        // Filter coefficient: alpha = dt / (RC + dt)
    float prev_output;  // Previous output value
} LPF_HandleTypeDef;

void LPF_Init(LPF_HandleTypeDef *hlpf, float cutoff_freq, float sampling_freq);

float LPF_Update(LPF_HandleTypeDef *hlpf, float input);

void LPF_Reset(LPF_HandleTypeDef *hlpf);

#endif /* LPF_H */
