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
#include "can_manager.h"
#include "error_codes.h"
#include "secondary_network.h"

int acquisinatore_can_id;
// #define CAN_MGR_N_MESSAGES 0
// can_mgr_msg_t can_message_states[CAN_MGR_N_MESSAGES];
// uint8_t can_message_new[CAN_MGR_N_MESSAGES];

int can_mgr_from_id_to_index(int can_id, int msg_id) {
  if (can_id != acquisinatore_can_id) {
    return -1;
  } else {
    return -1;
  }
}

#define CANLIB_PACK(msg_name, MSG_NAME, ntw, NTW)                              \
  can_mgr_msg_t msg_to_be_sent = {0};                                          \
  msg_to_be_sent.id = NTW##_##MSG_NAME##_FRAME_ID;                             \
  msg_to_be_sent.size = NTW##_##MSG_NAME##_BYTE_SIZE;                          \
  ntw##_##msg_name##_t raw = {0};                                              \
  ntw##_##msg_name##_conversion_to_raw_struct(&raw, &converted);               \
  ntw##_##msg_name##_pack(msg_to_be_sent.data, &raw, msg_to_be_sent.size);

void acquisinatore_send_cooling_temp(long double temperature) {
  secondary_cooling_temp_converted_t converted = {
  // .top_left = channel1 * 10,
  // .top_right = channel2 * 10,
#if ACQUISINATOR_ID == 1
    .bottom_right = temperature
#elif ACQUISINATOR_ID == 2
    .bottom_left = temperature
#else
    0
#endif
  };
  CANLIB_PACK(cooling_temp, COOLING_TEMP, secondary, SECONDARY);
  if (can_mgr_send(acquisinatore_can_id, &msg_to_be_sent) < 0) {
    acquisinatore_set_led_code(acquisinatore_led_code_can_not_working);
  }
}

void acquisinatore_send_strain_gauge_val(float strain_gauge_val) {
  secondary_rod_elongation_converted_t converted = {.deformation =
                                                        strain_gauge_val};
  CANLIB_PACK(rod_elongation, ROD_ELONGATION, secondary, SECONDARY);
  if (can_mgr_send(acquisinatore_can_id, &msg_to_be_sent) < 0) {
    acquisinatore_set_led_code(acquisinatore_led_code_can_not_working);
  }
}

void acquisinatore_send_raw_voltage_values(float channel1, float channel2) {
  secondary_debug_signal_converted_t converted = {
      .field_1 = (channel1 / 10.0f), .field_2 = (channel2 / 10.0f)};
  CANLIB_PACK(debug_signal, DEBUG_SIGNAL, secondary, SECONDARY);
  if (can_mgr_send(acquisinatore_can_id, &msg_to_be_sent) < 0) {
    acquisinatore_set_led_code(acquisinatore_led_code_can_not_working);
  }
}

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void) {

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_LOOPBACK;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  /*
  CAN_FilterTypeDef hfilter = {.FilterActivation = CAN_FILTER_ENABLE,
                                 .FilterBank = 0,
                                 .FilterFIFOAssignment = CAN_FILTER_FIFO0,
                                 .FilterIdHigh = ((1U << 11) - 1)
                                                 << 5, // Take all ids to 2^11 -
  1 .FilterIdLow = 0,     // Take all ids from 0 .FilterMaskIdHigh = 0,
                                 .FilterMaskIdLow = 0,
                                 .FilterMode = CAN_FILTERMODE_IDMASK,
                                 .FilterScale = CAN_FILTERSCALE_16BIT};
  */

  acquisinatore_can_id = can_mgr_init(&hcan);
  if (can_mgr_config(acquisinatore_can_id, NULL, 0, CAN_RX_FIFO0, NULL, NULL,
                     0) < 0) {
    HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
    Error_Handler();
  }
  if (can_mgr_start(acquisinatore_can_id) < 0) {
    HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
    Error_Handler();
  }

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
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
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
