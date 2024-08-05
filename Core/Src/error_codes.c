#include "error_codes.h"

uint32_t last_set_error                                  = acquisinatore_led_code_all_ok;
uint32_t error_countdown                                 = 0;
uint32_t previous_time                                   = 0;
uint32_t acquisinatore_led_on                            = 0;
uint32_t all_set_errors[acquisinatore_led_code_n_values] = {0};

#define LED_TOGGLING_WAITING_TIME (250U)  // in ms
#define WAITING_PERIOD            (LED_TOGGLING_WAITING_TIME * 3)

void acquisinatore_set_led_code(acquisinatore_led_code_t error_code) {
    if (error_code == acquisinatore_led_code_can_not_working) {
        system_reset();
    }
    all_set_errors[error_code] = 1;
    last_set_error             = error_code;
    if (((error_countdown == acquisinatore_led_code_no_error && (get_timestamp_ms() - previous_time) > WAITING_PERIOD))) {
        error_countdown      = error_code * 2;
        previous_time        = get_timestamp_ms();
        acquisinatore_led_on = 0;
        acquisinatore_turn_led(acquisinatore_led_on);
    }
}

void acquisinatore_led_code_routine(void) {
    acquisinatore_set_led_code(last_set_error);
    if (error_countdown > 0) {
        uint32_t current_time = get_timestamp_ms();
        uint32_t waiting_time = LED_TOGGLING_WAITING_TIME;
        if ((current_time - previous_time) >= waiting_time) {
            previous_time        = get_timestamp_ms();
            acquisinatore_led_on = !acquisinatore_led_on;
            acquisinatore_turn_led(acquisinatore_led_on);
            error_countdown--;
        }
    }
}
