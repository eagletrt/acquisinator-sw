#ifndef LPF_H
#define LPF_H

struct LPFHandler {
    float alpha;       // Filter coefficient: alpha = dt / (RC + dt)
    float prev_output; // Previous output value
};

#endif /* LPF_H */
