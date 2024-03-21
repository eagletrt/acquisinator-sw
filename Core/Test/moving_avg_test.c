#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <acquisinator_config.h>
#include "lpfilter.h"

#define MOVING_AVG_WINDOW 10


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
  return 0;
}

