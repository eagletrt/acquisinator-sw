#ifndef ERROR_CODES_H
#define ERROR_CODES_H

#include "acquisinator_config.h"

#include <stdint.h>

void acquisinatore_set_led_code(acquisinatore_led_code_t error_code);
void acquisinatore_led_code_routine(void);

#endif  // ERROR_CODES_H
