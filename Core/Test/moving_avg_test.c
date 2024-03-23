#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <acquisinator_config.h>
#include "lpfilter.h"

#define MOVING_AVG_WINDOW 10

#define LTC1865_N_CHANNELS 4

uint16_t acquisinatore_current_mean[LTC1865_N_CHANNELS];
uint16_t acquisinatore_history[LTC1865_N_CHANNELS][MOVING_AVG_WINDOW];
uint64_t acquisinatore_hsum[LTC1865_N_CHANNELS];
size_t acquisinatore_history_len[LTC1865_N_CHANNELS];
int acquisinatore_completed_loop[LTC1865_N_CHANNELS];

int main(int argc, char** argv) {
  uint16_t current_mean = 0;
  uint16_t history[MOVING_AVG_WINDOW] = {0};
  uint64_t hsum = 0;
  size_t history_len = 0;
  int completed_loop = 0;
  FILE* out = fopen("filtered.csv", "w+");
  srand(time(NULL));
  for (size_t i = 0; i < 500; i++) {
    uint16_t random_noise = 5000 + (uint16_t) (rand() % 200) - 100;
    uint16_t nv = apply_moving_avg(random_noise, &current_mean, history, MOVING_AVG_WINDOW, &hsum, &history_len, &completed_loop);
    fprintf(out, "%u,%u\n", (unsigned int) random_noise, (unsigned int) nv);
  }

  for (size_t i=0 ; i < 500; i++) {
    uint16_t random_noise = 5000 + (uint16_t) (rand() % 200) - 100;
    uint16_t nv = apply_moving_avg(random_noise, &current_mean, history, MOVING_AVG_WINDOW, &hsum, &history_len, &completed_loop);
    
    uint16_t nnv = apply_moving_avg(
      ltc1865_raw_values[channel], &acquisinatore_current_mean[channel],
      acquisinatore_history[channel], MOVING_AVG_WINDOW,
      &acquisinatore_hsum[channel], &acquisinatore_history_len[channel],
      &acquisinatore_completed_loop[channel]
    );
    fprintf(out, "%u,%u\n", (unsigned int) random_noise, (unsigned int) nv);
  }

  return 0;
}

