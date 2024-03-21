#include "lpfilter.h"

// static uint16_t current_mean = 0;
// static uint16_t history[MOVING_AVG_WINDOW] = {0};
// static uint64_t hsum = 0;
// static size_t history_len = 0;
// static int completed_loop = 0;

uint16_t apply_moving_avg(uint16_t new_value, uint16_t *mv_avg_current_mean,
                          uint16_t *history, const size_t moving_avg_window,
                          uint64_t *mv_avg_hsum, size_t *mv_avg_history_len,
                          int *mv_avg_completed_loop) {
  if ((*mv_avg_history_len) == moving_avg_window) {
    (*mv_avg_completed_loop) = 1;
  }
  (*mv_avg_history_len) = (*mv_avg_history_len) % moving_avg_window;
  uint16_t old_value = history[(*mv_avg_history_len)];
  history[(*mv_avg_history_len)] = new_value;
  ++(*mv_avg_history_len);
  (*mv_avg_hsum) += (new_value - old_value);
  if ((*mv_avg_completed_loop)) {
    return (*mv_avg_hsum) / moving_avg_window;
  } else {
    return new_value;
  }
}
