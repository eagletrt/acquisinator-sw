#ifndef ACQUISINATOR_CONFIG_H
#define ACQUISINATOR_CONFIG_H

#include <stdint.h>

#define ACQUISINATOR_ID_0  (0U)  // for strain gauge and air outside the radiators
#define ACQUISINATOR_ID_1  (1U)  // for the radiators cooling temperatures, water in and out of the radiators
#define ACQUISINATOR_ID_2  (2U)  // only strain gauge fl
#define ACQUISINATOR_ID_3  (3U)  // only strain gauge fr
#define ACQUISINATOR_ID_4  (4U)  // only strain gauge rr
#define ACQUISINATOR_ID_5  (5U)
#define ACQUISINATOR_ID_6  (6U)
#define ACQUISINATOR_ID_7  (7U)
#define ACQUISINATOR_ID_8  (8U)
#define ACQUISINATOR_ID_9  (9U)
#define ACQUISINATOR_ID_10 (10U)
#define ACQUISINATOR_ID_11 (11U)
#define ACQUISINATOR_ID_12 (12U)
#define ACQUISINATOR_ID_13 (13U)
#define ACQUISINATOR_ID_14 (14U)
#define ACQUISINATOR_ID_15 (15U)
#define ACQUISINATOR_ID_16 (16U)
#define ACQUISINATOR_ID_17 (17U)
#define ACQUISINATOR_ID_18 (18U)
#define ACQUISINATOR_ID_19 (19U)
#define ACQUISINATOR_ID_20 (20U)
#define ACQUISINATOR_ID_21 (21U)
#define ACQUISINATOR_ID_22 (22U)
#define ACQUISINATOR_ID_23 (23U)
#define ACQUISINATOR_ID_24 (24U)
#define ACQUISINATOR_ID_25 (25U)
#define ACQUISINATOR_ID_26 (26U)
#define ACQUISINATOR_ID_27 (27U)
#define ACQUISINATOR_ID_28 (28U)
#define ACQUISINATOR_ID_29 (29U)
#define ACQUISINATOR_ID_30 (30U)
#define ACQUISINATOR_ID_31 (31U)

#define ACQUISINATOR_ID (ACQUISINATOR_ID_2)

/****
 * Memory mapping:
 * 
 * 16KB openblt
 * 28KB user program
 * 2KB  configurations
 * 18KB crashdebug reserved space: ram dump
*/

#define OPENBLT_RESERVED_ADDRESS_0           (0x8000000U)
#define OPENBLT_RESERVED_ADDRESS_1           (0x8000800U)
#define OPENBLT_RESERVED_ADDRESS_2           (0x8001000U)
#define OPENBLT_RESERVED_ADDRESS_3           (0x8001800U)
#define OPENBLT_RESERVED_ADDRESS_4           (0x8002000U)
#define OPENBLT_RESERVED_ADDRESS_5           (0x8002800U)
#define OPENBLT_RESERVED_ADDRESS_6           (0x8003000U)
#define OPENBLT_RESERVED_ADDRESS_7           (0x8003800U)
#define USER_PROGRAM_RESERVED_ADDRESS_0      (0x8004000U)
#define USER_PROGRAM_RESERVED_ADDRESS_1      (0x8004800U)
#define USER_PROGRAM_RESERVED_ADDRESS_2      (0x8005000U)
#define USER_PROGRAM_RESERVED_ADDRESS_3      (0x8005800U)
#define USER_PROGRAM_RESERVED_ADDRESS_4      (0x8006000U)
#define USER_PROGRAM_RESERVED_ADDRESS_5      (0x8006800U)
#define USER_PROGRAM_RESERVED_ADDRESS_6      (0x8007000U)
#define USER_PROGRAM_RESERVED_ADDRESS_7      (0x8007800U)
#define USER_PROGRAM_RESERVED_ADDRESS_8      (0x8008000U)
#define USER_PROGRAM_RESERVED_ADDRESS_9      (0x8008800U)
#define USER_PROGRAM_RESERVED_ADDRESS_10     (0x8009000U)
#define USER_PROGRAM_RESERVED_ADDRESS_11     (0x8009800U)
#define USER_PROGRAM_RESERVED_ADDRESS_12     (0x800A000U)
#define USER_PROGRAM_RESERVED_ADDRESS_13     (0x800A800U)
#define ACQUISINATOR_CONFIG_RESERVED_ADDRESS (0x800B000U)
#define CRASH_DEBUG_RESERVED_ADDRESS_0       (0x800B800U)
#define CRASH_DEBUG_RESERVED_ADDRESS_1       (0x800C000U)
#define CRASH_DEBUG_RESERVED_ADDRESS_2       (0x800C800U)
#define CRASH_DEBUG_RESERVED_ADDRESS_3       (0x800D000U)
#define CRASH_DEBUG_RESERVED_ADDRESS_4       (0x800D800U)
#define CRASH_DEBUG_RESERVED_ADDRESS_5       (0x800E000U)
#define CRASH_DEBUG_RESERVED_ADDRESS_6       (0x800E800U)
#define CRASH_DEBUG_RESERVED_ADDRESS_7       (0x800F000U)
#define CRASH_DEBUG_RESERVED_ADDRESS_8       (0x800F800U)

typedef enum { ltc1865_DIFF, ltc1865_DIFF_INVERTED, ltc1865_SE_CH1, ltc1865_SE_CH2, LTC1865_N_CHANNELS } ltc1865_channel_t;

#define ACQUISINATORE_NO_FILTER         (0U)
#define ACQUISINATORE_SIMPLE_MOVING_AVG (1U)
#define ACQUISINATORE_GAUSSIAN_FILTER   (2U)
#define ACQUISINATORE_OLD_MOVING_AVG    (3U)
#define ACQUISINATORE_FILTER_TYPE       ACQUISINATORE_SIMPLE_MOVING_AVG

enum {
    ACQUISINATOR_SECONDARY_ACQUISINATOR_JMP_TO_BLT = 0,
    ACQUISINATOR_SECONDARY_LINK_DEFORMATION_SET_CALIBRATION,
    ACQUISINATOR_SECONDARY_AMMO_POS_SET_CALIBRATION,
    N_MONITORED_MESSAGES
};

#define CAN_MESSAGES_HANDLERS \
    { secondary_acquisinator_jmp_to_blt_handler, secondary_link_deformation_set_calibration_handler }

#define LINK_DEFORMATION_CALIBRATION_VALUE (1.5010000000001358f)

typedef enum {
    acquisinatore_led_code_no_error = 0,
    acquisinatore_led_code_all_ok,
    acquisinatore_led_code_can_not_working,
    acquisinatore_led_code_spi_error,
    acquisinatore_led_code_read_write_flash,
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

#define CANLIB_PACK_AND_SEND(msg_name, MSG_NAME, ntw, NTW)                   \
    can_mgr_msg_t msg_to_be_sent = {0};                                      \
    msg_to_be_sent.id            = NTW##_##MSG_NAME##_FRAME_ID;              \
    msg_to_be_sent.size          = NTW##_##MSG_NAME##_BYTE_SIZE;             \
    ntw##_##msg_name##_t raw     = {0};                                      \
    ntw##_##msg_name##_conversion_to_raw_struct(&raw, &converted);           \
    ntw##_##msg_name##_pack(msg_to_be_sent.data, &raw, msg_to_be_sent.size); \
    if (can_mgr_send(acquisinatore_can_id, &msg_to_be_sent) < 0) {           \
        acquisinatore_set_led_code(acquisinatore_led_code_can_not_working);  \
    }

#endif
