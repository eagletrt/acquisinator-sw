#ifndef ACQUISINATOR_CONFIG_H
#define ACQUISINATOR_CONFIG_H

#define ACQUISINATOR_ID 1

#include <stdint.h>

/***
 * NB: LED always on = error on CAN bus initialization
 */

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

// #define NTC_COOLING_CONV(val)                                                                                                    \
//     (144.753928685635 + (-441.361483066664 * val) + (808.342944901787 * powl(val, 2.0)) + (-831.830860786940 * powl(val, 3.0)) + \
//      (425.004221588082 * powl(val, 4.0)) + (-84.5939048598991 * powl(val, 5.0)))

#define NTC_COOLING_DELAY_MS (200U)
#define ROD_DELAY_MS         (1U)

#define STRAIN_GAUGE_R1   (352)
#define STRAIN_GAUGE_R2   (350)
#define STRAIN_GAUGE_R3   (352)
#define STRAIN_GAUGE_RNOM (350)
#define STRAIN_GAUGE_G    (100)
#define STRAIN_GAUGE_VREF (1.501f)
#define STRAIN_GAUGE_VS   (3.294)
#define STRAIN_GAUGE_K    (2.01)
// #define FROM_V_TO_ROD_ELONGATION(Vval)                                                                                                     \
//     ((STRAIN_GAUGE_RNOM -                                                                                                                  \
//       ((STRAIN_GAUGE_R2 * STRAIN_GAUGE_R3) -                                                                                               \
//        (STRAIN_GAUGE_R2 * ((Vval - STRAIN_GAUGE_VREF) / STRAIN_GAUGE_G) * (STRAIN_GAUGE_R1 + STRAIN_GAUGE_R2)) / STRAIN_GAUGE_VS) /        \
//           (STRAIN_GAUGE_R1 + ((((Vval - STRAIN_GAUGE_VREF) / STRAIN_GAUGE_G) * (STRAIN_GAUGE_R1 + STRAIN_GAUGE_R2)) / STRAIN_GAUGE_VS))) / \
//      (STRAIN_GAUGE_RNOM * STRAIN_GAUGE_K))

#endif
