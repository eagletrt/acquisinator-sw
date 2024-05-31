/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

#include "acquisinator_config.h"
#include "acquisinatore.h"
#include "can_manager.h"
#include "error_codes.h"

int acquisinatore_can_id;

can_mgr_msg_t can_messages_states[N_MONITORED_MESSAGES];
uint8_t can_messages_is_new[N_MONITORED_MESSAGES];

int can_mgr_from_id_to_index(int can_id, int msg_id) {
    if (can_id != acquisinatore_can_id)
        return -1;
    switch (msg_id) {
        case SECONDARY_ACQUISINATOR_JMP_TO_BLT_FRAME_ID:
            return ACQUISINATOR_SECONDARY_ACQUISINATOR_JMP_TO_BLT;
        case SECONDARY_LINK_DEFORMATION_SET_CALIBRATION_FRAME_ID:
            return ACQUISINATOR_SECONDARY_LINK_DEFORMATION_SET_CALIBRATION;
        case SECONDARY_AMMO_COMPRESSION_SET_CALIBRATION_FRAME_ID:
            return ACQUISINATOR_SECONDARY_AMMO_POS_SET_CALIBRATION;
        default:
            return -1;
    }
    return -1;
}

int secondary_acquisinator_jmp_to_blt_handler(can_mgr_msg_t *msg) {
    secondary_acquisinator_jmp_to_blt_t jmp_raw;
    secondary_acquisinator_jmp_to_blt_converted_t jmp_converted;
    secondary_acquisinator_jmp_to_blt_unpack(&jmp_raw, msg->data, SECONDARY_ACQUISINATOR_JMP_TO_BLT_BYTE_SIZE);
    secondary_acquisinator_jmp_to_blt_raw_to_conversion_struct(&jmp_converted, &jmp_raw);
    if (jmp_converted.acquisinatore_id == ACQUISINATOR_ID) {
        HAL_NVIC_SystemReset();
    }
    return 0;
}

extern bool calibration_ammo_pos_flag;
extern bool calibration_reset_link_deformation_flag;

int secondary_ammo_pos_set_calibration_handler(can_mgr_msg_t *msg) {
    calibration_ammo_pos_flag = true;
    return 0;
}

int secondary_link_deformation_set_calibration_handler(can_mgr_msg_t *msg) {
    calibration_reset_link_deformation_flag = true;
    return 0;
}

int (*primary_message_handlers[N_MONITORED_MESSAGES])(can_mgr_msg_t *) = CAN_MESSAGES_HANDLERS;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    can_mgr_it_callback(hcan, CAN_RX_FIFO0, NULL);
}

void acquisinatore_send_water_cooling_temp(double radiator_input, double radiator_output) {
    secondary_cooling_temp_pumps_converted_t converted = {.input = radiator_input, .output = radiator_output};
    CANLIB_PACK_AND_SEND(cooling_temp_pumps, COOLING_TEMP_PUMPS, secondary, SECONDARY);
}

void acquisinatore_send_air_cooling_temp(double air_temperature) {
    secondary_cooling_temp_radiators_converted_t converted = {.air_temp = air_temperature};
    CANLIB_PACK_AND_SEND(cooling_temp_radiators, COOLING_TEMP_RADIATORS, secondary, SECONDARY);
}

void acquisinatore_send_strain_gauge_val_fl_wheel(uint8_t rod_id, float strain_gauge_val) {
    secondary_link_deformation_fl_wheel_converted_t converted = {.rod_id = rod_id, .deformation = strain_gauge_val};
    CANLIB_PACK_AND_SEND(link_deformation_fl_wheel, LINK_DEFORMATION_FL_WHEEL, secondary, SECONDARY);
}

void acquisinatore_send_strain_gauge_val_fr_wheel(uint8_t rod_id, float strain_gauge_val) {
    secondary_link_deformation_fr_wheel_converted_t converted = {.rod_id = rod_id, .deformation = strain_gauge_val};
    CANLIB_PACK_AND_SEND(link_deformation_fr_wheel, LINK_DEFORMATION_FR_WHEEL, secondary, SECONDARY);
}

void acquisinatore_send_strain_gauge_val_rl_wheel(uint8_t rod_id, float strain_gauge_val) {
    secondary_link_deformation_rl_wheel_converted_t converted = {.rod_id = rod_id, .deformation = strain_gauge_val};
    CANLIB_PACK_AND_SEND(link_deformation_rl_wheel, LINK_DEFORMATION_RL_WHEEL, secondary, SECONDARY);
}

void acquisinatore_send_strain_gauge_val_rr_wheel(uint8_t rod_id, float strain_gauge_val) {
    secondary_link_deformation_rr_wheel_converted_t converted = {.rod_id = rod_id, .deformation = strain_gauge_val};
    CANLIB_PACK_AND_SEND(link_deformation_rr_wheel, LINK_DEFORMATION_RR_WHEEL, secondary, SECONDARY);
}

void acquisinatore_send_calibration_offsets(float off1, float off2) {
    secondary_acquisinator_calibrations_offsets_converted_t converted = { .acquisinator_id = ACQUISINATOR_ID, .offset1 = off1, .offset2 = off2 };
    CANLIB_PACK_AND_SEND(acquisinator_calibrations_offsets, ACQUISINATOR_CALIBRATIONS_OFFSETS, secondary, SECONDARY);
}

void acquisinatore_send_debug_1_values(float v1, float v2, float v3, float v4) {
    secondary_debug_signal_1_converted_t converted = {.field_1 = v1, .field_2 = v2, .field_3 = v3, .field_4 = v4};
    CANLIB_PACK_AND_SEND(debug_signal_1, DEBUG_SIGNAL_1, secondary, SECONDARY);
}

void acquisinatore_send_debug_2_values(float v1, float v2, float v3, float v4) {
    secondary_debug_signal_2_converted_t converted = {.field_1 = v1, .field_2 = v2, .field_3 = v3, .field_4 = v4};
    CANLIB_PACK_AND_SEND(debug_signal_2, DEBUG_SIGNAL_2, secondary, SECONDARY);
}

void acquisinatore_send_version(void) {
    secondary_acquisinator_version_converted_t converted = {.acquisinator_id = ACQUISINATOR_ID, .canlib_build_time = CANLIB_BUILD_TIME};
    CANLIB_PACK_AND_SEND(acquisinator_version, ACQUISINATOR_VERSION, secondary, SECONDARY);
}

int can_routine(void) {
    for (size_t msg_idx = 0; msg_idx < N_MONITORED_MESSAGES; ++msg_idx) {
        if (can_messages_is_new[msg_idx] && (primary_message_handlers[msg_idx] != NULL)) {
            can_messages_is_new[msg_idx] = 0;
            (*primary_message_handlers[msg_idx])(&can_messages_states[msg_idx]);
        }
    }
    return 0;
}

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void) {
    /* USER CODE BEGIN CAN_Init 0 */

    /* USER CODE END CAN_Init 0 */

    /* USER CODE BEGIN CAN_Init 1 */

    /* USER CODE END CAN_Init 1 */
    hcan.Instance                  = CAN;
    hcan.Init.Prescaler            = 2;
    hcan.Init.Mode                 = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1             = CAN_BS1_15TQ;
    hcan.Init.TimeSeg2             = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode    = DISABLE;
    hcan.Init.AutoBusOff           = DISABLE;
    hcan.Init.AutoWakeUp           = DISABLE;
    hcan.Init.AutoRetransmission   = DISABLE;
    hcan.Init.ReceiveFifoLocked    = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN_Init 2 */
    CAN_FilterTypeDef primary_filter = {
        .FilterMode           = CAN_FILTERMODE_IDMASK,
        .FilterIdLow          = 0 << 5,                 // Take all ids from 0
        .FilterIdHigh         = ((1U << 11) - 1) << 5,  // to 2^11 - 1
        .FilterMaskIdHigh     = 0 << 5,                 // Don't care on can id bits
        .FilterMaskIdLow      = 0 << 5,                 // Don't care on can id bits
        .FilterFIFOAssignment = CAN_FILTER_FIFO0,
        .FilterBank           = 0,
        .FilterScale          = CAN_FILTERSCALE_16BIT,
        .FilterActivation     = ENABLE,
        .SlaveStartFilterBank = 0};
    acquisinatore_can_id = can_mgr_init(&hcan);
    if (acquisinatore_can_id < 0) {
        Error_Handler();
        HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
    }
    if (can_mgr_config(
            acquisinatore_can_id,
            &primary_filter,
            CAN_IT_RX_FIFO0_MSG_PENDING,
            CAN_RX_FIFO0,
            can_messages_states,
            can_messages_is_new,
            N_MONITORED_MESSAGES) < 0) {
        Error_Handler();
        HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
    }

    if (can_mgr_start(acquisinatore_can_id) < 0) {
        Error_Handler();
        HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
    }

#if 0
    // Old configuration only sending messages
    CAN_FilterTypeDef hfilter = {.FilterActivation = CAN_FILTER_ENABLE,
                               .FilterBank = 0,
                               .FilterFIFOAssignment = CAN_FILTER_FIFO0,
                               .FilterIdHigh = ((1U << 11) - 1) << 5, // Take all ids to 2^11 - 1 
                               .FilterIdLow = 0,     // Take all ids from 0 .FilterMaskIdHigh = 0,
                               .FilterMaskIdLow = 0,
                               .FilterMode = CAN_FILTERMODE_IDMASK,
                               .FilterScale = CAN_FILTERSCALE_16BIT};

    acquisinatore_can_id = can_mgr_init(&hcan);
    if (can_mgr_config(acquisinatore_can_id, NULL, 0, CAN_RX_FIFO0, NULL, NULL, 0) < 0) {
        HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
        Error_Handler();
    }
    if (can_mgr_start(acquisinatore_can_id) < 0) {
        HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
    }
#endif

    /* USER CODE END CAN_Init 2 */
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (canHandle->Instance == CAN) {
        /* USER CODE BEGIN CAN_MspInit 0 */

        /* USER CODE END CAN_MspInit 0 */
        /* CAN clock enable */
        __HAL_RCC_CAN1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
        GPIO_InitStruct.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* CAN interrupt Init */
        HAL_NVIC_SetPriority(CAN_TX_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(CAN_TX_IRQn);
        HAL_NVIC_SetPriority(CAN_RX0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(CAN_RX0_IRQn);
        HAL_NVIC_SetPriority(CAN_RX1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(CAN_RX1_IRQn);
        /* USER CODE BEGIN CAN_MspInit 1 */

        /* USER CODE END CAN_MspInit 1 */
    }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle) {
    if (canHandle->Instance == CAN) {
        /* USER CODE BEGIN CAN_MspDeInit 0 */

        /* USER CODE END CAN_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_CAN1_CLK_DISABLE();

        /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

        /* CAN interrupt Deinit */
        HAL_NVIC_DisableIRQ(CAN_TX_IRQn);
        HAL_NVIC_DisableIRQ(CAN_RX0_IRQn);
        HAL_NVIC_DisableIRQ(CAN_RX1_IRQn);
        /* USER CODE BEGIN CAN_MspDeInit 1 */

        /* USER CODE END CAN_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
