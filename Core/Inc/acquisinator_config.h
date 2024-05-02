#ifndef ACQUISINATOR_CONFIG_H
#define ACQUISINATOR_CONFIG_H

#include <stdint.h>

typedef enum { ltc1865_DIFF, ltc1865_DIFF_INVERTED, ltc1865_SE_CH1, ltc1865_SE_CH2, LTC1865_N_CHANNELS } ltc1865_channel_t;

#define ACQUISINATOR_ID 1

#define ACQUISINATORE_FILTER_TYPE       ACQUISINATORE_SIMPLE_MOVING_AVG
#define ACQUISINATORE_NO_FILTER         (0u)
#define ACQUISINATORE_SIMPLE_MOVING_AVG (1u)
#define ACQUISINATORE_GAUSSIAN_FILTER   (2u)
#define ACQUISINATORE_OLD_MOVING_AVG    (3u)

#if ACQUISINATOR_ID == 1
#define TOP_LEFT_NTC_TEMPERATURE_CALIBRATION_THRESHOLD (0.0f)
#define ROD_ELONGATION_CALIBRATION_THRESHOLD           (0.0f)
#elif ACQUISINATOR_ID == 2
#define TOP_RIGHT_NTC_TEMPERATURE_CALIBRATION_THRESHOLD    (0.0f)
#define BOTTOM_RIGHT_NTC_TEMPERATURE_CALIBRATION_THRESHOLD (0.0f)
#endif

enum { ACQUISINATOR_SECONDARY_ACQUISINATOR_JMP_TO_BLT = 0, ACQUISINATOR_SECONDARY_LINK_DEFORMATION_SET_CALIBRATION, N_MONITORED_MESSAGES };

#define CAN_MESSAGES_HANDLERS \
    { secondary_acquisinator_jmp_to_blt_handler, secondary_link_deformation_set_calibration_handler }

typedef enum {
    acquisinatore_led_code_no_error = 0,
    acquisinatore_led_code_all_ok,
    acquisinatore_led_code_can_not_working,
    acquisinatore_led_code_spi_error,
    acquisinatore_led_code_n_values
} acquisinatore_led_code_t;

void acquisinatore_turn_led(int on);
uint32_t get_timestamp_ms(void);

#define ACQUISINATORE_VREF_INT (3.29f)

#define NTC_COOLING_DELAY_MS (200U)
#define ROD_DELAY_MS         (1U)

#define STRAIN_GAUGE_R1   (352.0f)
#define STRAIN_GAUGE_R2   (350.0f)
#define STRAIN_GAUGE_R3   (352.0f)
#define STRAIN_GAUGE_RNOM (350.0f)
#define STRAIN_GAUGE_G    (100.0f)
#define STRAIN_GAUGE_VREF (1.501f)
#define STRAIN_GAUGE_VS   (3.294f)
#define STRAIN_GAUGE_K    (2.01f)

#endif
