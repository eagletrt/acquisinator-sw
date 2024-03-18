#include "acquisinator_config.h"
#include "error_codes.h"
#include <stdio.h>
#define _POSIX_C_SOURCE 200809L

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <time.h>

void acquisinatore_turn_led(int on) {
  if (on) {
    printf("Acceso\n");
  } else {
    printf("Spento\n");
  }
}

uint32_t get_timestamp_ms(void) {
  long ms;
  time_t s;
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  s = spec.tv_sec;
  ms = round(spec.tv_nsec / 1.0e6);
  return ((s % 100) * 1000 + ms);
}

extern uint32_t last_set_error;
int main(int argc, char **argv) {
  uint32_t previous_time = get_timestamp_ms();
  int done = 0;
  acquisinatore_set_led_code(acquisinatore_led_code_all_ok);
  while (1) {
    acquisinatore_set_led_code(last_set_error);
    if ((get_timestamp_ms() - previous_time) > 2000 && !done) {
      done = 1;
      acquisinatore_set_led_code(acquisinatore_led_code_spi_error);
    }

    acquisinatore_led_code_routine();
  }
  return 0;
}
