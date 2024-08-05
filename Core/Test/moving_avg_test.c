#include "acquisinator_config.h"
#include "acquisinatore.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define MOVING_AVG_WINDOW (10U)
#define N_FAKE_VALUES     (2000U)

#define LTC1865_N_CHANNELS (4U)

#if ACQUISINATORE_FILTER_TYPE == ACQUISINATORE_SIMPLE_MOVING_AVG

double vabom[N_FAKE_VALUES];

#elif ACQUISINATORE_FILTER_TYPE == ACQUISINATORE_GAUSSIAN_FILTER

double vabom[N_FAKE_VALUES];
double vabom2[N_FAKE_VALUES];
double acquisinatore_kernel[ACQUISINATORE_GAUSSIAN_KERNEL_SIZE] = {0};

#elif ACQUISINATORE_FILTER_TYPE == ACQUISINATORE_OLD_MOVING_AVG

uint16_t current_mean               = 0;
uint16_t history[MOVING_AVG_WINDOW] = {0};
uint64_t hsum                       = 0;
size_t history_len                  = 0;
int completed_loop                  = 0;
int channel                         = 0;

uint16_t acquisinatore_current_mean[LTC1865_N_CHANNELS];
uint16_t acquisinatore_history[LTC1865_N_CHANNELS][MOVING_AVG_WINDOW];
uint64_t acquisinatore_hsum[LTC1865_N_CHANNELS];
size_t acquisinatore_history_len[LTC1865_N_CHANNELS];
int acquisinatore_completed_loop[LTC1865_N_CHANNELS];

#endif

uint16_t moving_avg(uint16_t *A, size_t L) {
    double tot = 0.0;
    for (size_t i = 0; i < L; i++) {
        tot += (double)A[i] / L;
    }
    return (uint16_t)tot;
}

int main(int argc, char **argv) {
    FILE *out = fopen("filtered.csv", "w+");
    srand(time(NULL));

#if ACQUISINATORE_FILTER_TYPE == ACQUISINATORE_SIMPLE_MOVING_AVG
    for (size_t i = 0; i < N_FAKE_VALUES; i++) {
        uint16_t random_noise                                          = 5000 + (uint16_t)(rand() % 200) - 100;
        #warning Fix the moving avg test
        // TODO: fix
        // acquisinatore_mov_avg_window[acquisinatore_mov_avg_window_idx] = (double)random_noise;
        // acquisinatore_mov_avg_window_idx = (acquisinatore_mov_avg_window_idx + 1) % ACQUISINATORE_SIMPLE_MOVING_AVG_KERNEL_SIZE;
        vabom[i]                         = (double)random_noise;
        double filtered_val              = moving_avg(acquisinatore_mov_avg_window, ACQUISINATORE_SIMPLE_MOVING_AVG_KERNEL_SIZE);
        fprintf(out, "%f,%f\n", vabom[i], filtered_val);
    }

#elif ACQUISINATORE_FILTER_TYPE == ACQUISINATORE_GAUSSIAN_FILTER

    lpf_gaussian_init(kernel, ACQUISINATORE_GAUSSIAN_KERNEL_SIZE, 1.8);
    uint8_t vabom3idx = 0;

    for (size_t i = 0; i < N_FAKE_VALUES; i++) {
        uint16_t random_noise                   = 5000 + (uint16_t)(rand() % 200) - 100;
        acquisinatore_mov_avg_window[vabom3idx] = (double)random_noise;
        vabom3idx                               = (vabom3idx + 1) % CIRCULAR_BUFFER_SIZE;
        vabom[i] = vabom2[i] = (double)random_noise;
        if (i > ACQUISINATORE_GAUSSIAN_KERNEL_SIZE) {
            vabom2[i - (ACQUISINATORE_GAUSSIAN_KERNEL_SIZE / 2)] =
                lpf_gaussian_apply_iir(acquisinatore_mov_avg_window, acquisinatore_kernel, CIRCULAR_BUFFER_SIZE);
            fprintf(
                out, "%f,%f\n", vabom[i - (ACQUISINATORE_GAUSSIAN_KERNEL_SIZE / 2)], vabom2[i - (ACQUISINATORE_GAUSSIAN_KERNEL_SIZE / 2)]);
        }
    }
#endif

    return 0;
}
