#ifndef LPFILTER_H
#define LPFILTER_H

#include "acquisinator_config.h"

#include <stddef.h>
#include <stdint.h>

#if ACQUISINATORE_FILTER_TYPE == ACQUISINATORE_SIMPLE_MOVING_AVG

#define ACQUISINATORE_SIMPLE_MOVING_AVG_KERNEL_SIZE (10U)

extern uint16_t acquisinatore_mov_avg_window[ACQUISINATORE_SIMPLE_MOVING_AVG_KERNEL_SIZE];
extern uint16_t acquisinatore_kernel[ACQUISINATORE_SIMPLE_MOVING_AVG_KERNEL_SIZE];
extern uint8_t acquisinatore_mov_avg_window_idx;
uint16_t moving_avg(uint16_t *, size_t);

#elif ACQUISINATORE_FILTER_TYPE == ACQUISINATORE_GAUSSIAN_FILTER

#define KERNEL_SIZE                        100
#define ACQUISINATORE_GAUSSIAN_KERNEL_SIZE (7U)

extern double sigma;
extern size_t size;
extern size_t half_size;
extern double kernel[KERNEL_SIZE];

void lpf_gaussian_init(double *const kernel, size_t size, const double standard_deviation);
void lpf_gaussian_apply(const double *input_buffer, double *output_buffer, size_t buffer_size);
double lpf_gaussian_apply_iir(const double *input_buffer, const double *kernel, size_t ksize);

#elif ACQUISINATORE_FILTER_TYPE == ACQUISINATORE_OLD_MOVING_AVG

uint16_t apply_moving_avg(
    uint16_t new_value,
    uint16_t *mv_avg_current_mean,
    uint16_t *history,
    const size_t moving_avg_window,
    uint64_t *mv_avg_hsum,
    size_t *mv_avg_history_len,
    int *mv_avg_completed_loop);

#endif

#endif  // LPFILTER_H
