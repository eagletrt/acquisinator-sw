#ifndef ACQUISINATOR_CONFIG_H
#define ACQUISINATOR_CONFIG_H

#include <stdint.h>

#define ACQUISINATOR_ID_0  (0)  // for strain gauge and air outside the radiators
#define ACQUISINATOR_ID_1  (1)  // for the radiators cooling temperatures, water in and out of the radiators
#define ACQUISINATOR_ID_2  (2)  // only strain gauge fl
#define ACQUISINATOR_ID_3  (3)  // only strain gauge fr
#define ACQUISINATOR_ID_4  (4)  // only strain gauge rr
#define ACQUISINATOR_ID_5  (5)
#define ACQUISINATOR_ID_6  (6)
#define ACQUISINATOR_ID_7  (7)
#define ACQUISINATOR_ID_8  (8)
#define ACQUISINATOR_ID_9  (9)
#define ACQUISINATOR_ID_10 (10)
#define ACQUISINATOR_ID_11 (11)
#define ACQUISINATOR_ID_12 (12)
#define ACQUISINATOR_ID_13 (13)
#define ACQUISINATOR_ID_14 (14)
#define ACQUISINATOR_ID_15 (15)
#define ACQUISINATOR_ID_16 (16)
#define ACQUISINATOR_ID_17 (17)
#define ACQUISINATOR_ID_18 (18)
#define ACQUISINATOR_ID_19 (19)
#define ACQUISINATOR_ID_20 (20)
#define ACQUISINATOR_ID_21 (21)
#define ACQUISINATOR_ID_22 (22)
#define ACQUISINATOR_ID_23 (23)
#define ACQUISINATOR_ID_24 (24)
#define ACQUISINATOR_ID_25 (25)
#define ACQUISINATOR_ID_26 (26)
#define ACQUISINATOR_ID_27 (27)
#define ACQUISINATOR_ID_28 (28)
#define ACQUISINATOR_ID_29 (29)
#define ACQUISINATOR_ID_30 (30)
#define ACQUISINATOR_ID_31 (31)

#define ACQUISINATOR_ID (ACQUISINATOR_ID_2)

typedef enum { ltc1865_DIFF, ltc1865_DIFF_INVERTED, ltc1865_SE_CH1, ltc1865_SE_CH2, LTC1865_N_CHANNELS } ltc1865_channel_t;

#define ACQUISINATORE_FILTER_TYPE       ACQUISINATORE_SIMPLE_MOVING_AVG
#define ACQUISINATORE_NO_FILTER         (0u)
#define ACQUISINATORE_SIMPLE_MOVING_AVG (1u)
#define ACQUISINATORE_GAUSSIAN_FILTER   (2u)
#define ACQUISINATORE_OLD_MOVING_AVG    (3u)

#if ACQUISINATOR_ID == ACQUISINATOR_ID_0
#define TOP_LEFT_NTC_TEMPERATURE_CALIBRATION_THRESHOLD (0.0f)
#define ROD_ELONGATION_CALIBRATION_THRESHOLD           (0.0f)
#elif ACQUISINATOR_ID == ACQUISINATOR_ID_1
#define TOP_RIGHT_NTC_TEMPERATURE_CALIBRATION_THRESHOLD    (0.0f)
#define BOTTOM_RIGHT_NTC_TEMPERATURE_CALIBRATION_THRESHOLD (0.0f)
#elif ACQUISINATOR_ID == ACQUISINATOR_ID_2
#define ROD_ELONGATION_CALIBRATION_THRESHOLD (0.0f)
#elif ACQUISINATOR_ID == ACQUISINATOR_ID_3
#define ROD_ELONGATION_CALIBRATION_THRESHOLD (0.0f)
#elif ACQUISINATOR_ID == ACQUISINATOR_ID_4
#define ROD_ELONGATION_CALIBRATION_THRESHOLD (0.0f)
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
