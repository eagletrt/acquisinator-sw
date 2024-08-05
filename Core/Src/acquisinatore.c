#include "acquisinatore.h"

#if ACQUISINATORE_FILTER_TYPE == ACQUISINATORE_SIMPLE_MOVING_AVG

uint16_t acquisinatore_mov_avg_window[LTC1865_N_CHANNELS][ACQUISINATORE_SIMPLE_MOVING_AVG_KERNEL_SIZE] = {0};
uint16_t acquisinatore_kernel[ACQUISINATORE_SIMPLE_MOVING_AVG_KERNEL_SIZE]                             = {0};
uint8_t acquisinatore_mov_avg_window_idx[LTC1865_N_CHANNELS]                                           = {0};

#elif ACQUISINATORE_FILTER_TYPE == ACQUISINATORE_GAUSSIAN_FILTER

#include "math.h"
#include "ring-buffer.h"

double sigma;
size_t size;
size_t half_size;
double kernel[KERNEL_SIZE];

void lpfilter_init() {
    lpf_gaussian_init(kernel, ACQUISINATORE_GAUSSIAN_KERNEL_SIZE, 1.8);
}

void lpf_gaussian_init(double *const kernel, size_t size, const double standard_deviation) {
    double sigma     = standard_deviation;
    size_t ksize     = size;
    size_t half_size = size / 2;

    double lambda = 0;
    for (size_t i = 0; i < size; ++i) {
        int k = (int)i - (int)half_size;
        lambda += (kernel[i] = exp(-(k * k) / (2.0 * sigma * sigma)));
    }
    lambda /= size;

    for (size_t i = 0; i < size; ++i)
        kernel[i] /= (lambda * size);
}

void lpf_gaussian_apply(const double *input_buffer, double *output_buffer, size_t buffer_size) {
    for (size_t i = half_size; buffer_size >= size && i < buffer_size - half_size; ++i) {
        output_buffer[i] = 0;
        for (size_t k = 0; k < size; ++k) {
            size_t index = i + k - half_size;
            output_buffer[i] += input_buffer[index] * kernel[k];
        }
    }
}

double lpf_gaussian_apply_iir(const double *input_buffer, const double *kernel, size_t ksize) {
    size_t half_size = ksize / 2;
    double val       = 0;
    for (size_t k = 0; k < ksize; ++k) {
        val += input_buffer[k] * kernel[k];
    }
    return val;
}

#elif ACQUISINATORE_FILTER_TYPE == ACQUISINATORE_OLD_MOVING_AVG

uint16_t apply_moving_avg(
    uint16_t new_value,
    uint16_t *mv_avg_current_mean,
    uint16_t *history,
    const size_t moving_avg_window,
    uint64_t *mv_avg_hsum,
    size_t *mv_avg_history_len,
    int *mv_avg_completed_loop) {
    if ((*mv_avg_history_len) == moving_avg_window) {
        (*mv_avg_completed_loop) = 1;
    }
    (*mv_avg_history_len)          = (*mv_avg_history_len) % moving_avg_window;
    uint16_t old_value             = history[(*mv_avg_history_len)];
    history[(*mv_avg_history_len)] = new_value;
    ++(*mv_avg_history_len);
    (*mv_avg_hsum) += (new_value - old_value);
    if ((*mv_avg_completed_loop)) {
        return (*mv_avg_hsum) / moving_avg_window;
    } else {
        return new_value;
    }
}

#endif
