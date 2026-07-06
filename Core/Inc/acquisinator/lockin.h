#ifndef LOCKIN_H
#define LOCKIN_H


#include <stdio.h>

// Struttura per una singola cella Biquad (Filtro del 2° ordine)
typedef struct {
    // Coefficienti del feedforward (Numeratore)
    float b0, b1, b2;
    // Coefficienti del feedback (Denominatore) - a0 è assunto = 1.0
    float a1, a2;
    // Memoria di stato (Delay lines)
    float z1, z2;
} Biquad;

// Struttura per il filtro IIR completo (cascata di più Biquad)
#define NUM_STAGES 1 // 1 per un Butterworth del 1° o 2° ordine. Aumenta per ordini superiori.

typedef struct {
    Biquad stages[NUM_STAGES];
} IIRFilter;

// Inizializza o resetta lo stato del filtro
static inline void IIRFilter_Init(IIRFilter *filt) {
    for (int i = 0; i < NUM_STAGES; i++) {
        filt->stages[i].z1 = 0.0f;
        filt->stages[i].z2 = 0.0f;
    }
}

// Elabora un singolo campione in ingresso (Direct Form II Transposed)
static inline float IIRFilter_Process(IIRFilter *filt, float in_sample) {
    float out = in_sample;
    for (int i = 0; i < NUM_STAGES; i++) {
        Biquad *bq = &filt->stages[i];
        float new_out = (bq->b0 * out) + bq->z1;
        bq->z1 = (bq->b1 * out) - (bq->a1 * new_out) + bq->z2;
        bq->z2 = (bq->b2 * out) - (bq->a2 * new_out);
        out = new_out;
    }
    return out;
}

#endif /* LOCKIN_H */