#ifndef LPFILTER_H
#define LPFILTER_H

#include <stddef.h>
#include <stdint.h>

uint16_t apply_moving_avg(
    uint16_t new_value,
    uint16_t *mv_avg_current_mean,
    uint16_t *history,
    const size_t moving_avg_window,
    uint64_t *mv_avg_hsum,
    size_t *mv_avg_history_len,
    int *mv_avg_completed_loop);

#endif  // LPFILTER_H
