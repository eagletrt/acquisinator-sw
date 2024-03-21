/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    spi.c
 * @brief   This file provides code for the configuration
 *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */

#include "lpfilter.h"

#define MOVING_AVG_WINDOW 10

uint16_t ltc1865_raw_values[LTC1865_N_CHANNELS];
uint16_t ltc1865_raw_filtered_values[LTC1865_N_CHANNELS];
float ltc1865_values_in_V[LTC1865_N_CHANNELS];

uint16_t acquisinatore_current_mean[LTC1865_N_CHANNELS];
uint16_t acquisinatore_history[LTC1865_N_CHANNELS][MOVING_AVG_WINDOW];
uint64_t acquisinatore_hsum[LTC1865_N_CHANNELS];
size_t acquisinatore_history_len[LTC1865_N_CHANNELS];
int acquisinatore_completed_loop[LTC1865_N_CHANNELS];

void wait_5us(void);

int ltc1865_select_channel(ltc1865_channel_t c) {
  uint8_t channel_selection_bits[2] = {0x00};
  switch (c) {
  case ltc1865_SE_CH1:
    channel_selection_bits[0] = 0xFF;
    channel_selection_bits[1] = 0xFF;
    break;
  case ltc1865_SE_CH2:
    channel_selection_bits[0] = 0xAA;
    channel_selection_bits[1] = 0xAA;
    break;
  case ltc1865_DIFF:
    channel_selection_bits[0] = 0x00;
    channel_selection_bits[1] = 0x00;
    break;
  case ltc1865_DIFF_INVERTED:
    channel_selection_bits[0] = 0x55;
    channel_selection_bits[1] = 0x55;
    break;
  default:
    break;
  }
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin, GPIO_PIN_SET);
  // wait_5us();
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef spi_trasm_res =
      HAL_SPI_Transmit(&hspi1, channel_selection_bits, 2, 100);
  if (spi_trasm_res == HAL_OK) {
    return 1;
  }
  return 0;
}

uint16_t ltc1865_spi_rcv(void) {
  uint16_t cell_value = 0;
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin, GPIO_PIN_SET);
  // wait_5us();
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Receive(&hspi1, (uint8_t *)&cell_value, 2, 100) != HAL_OK) {
    return -1;
  }
  return cell_value;
}

float ltc1865_read(ltc1865_channel_t channel) {
  if (ltc1865_select_channel(channel) == 0) {
    return -1;
  }
  uint16_t cval = ltc1865_spi_rcv();
  if (cval == -1) {
    return -1;
  }
  ltc1865_raw_values[channel] = (cval << 8) | (cval >> 8);
  ltc1865_raw_filtered_values[channel] = apply_moving_avg(
      ltc1865_raw_values[channel], &acquisinatore_current_mean[channel],
      acquisinatore_history[channel], MOVING_AVG_WINDOW,
      &acquisinatore_hsum[channel], &acquisinatore_history_len[channel],
      &acquisinatore_completed_loop[channel]
  );
  ltc1865_values_in_V[channel] =
      (ACQUISINATORE_VREF_INT * ((float)ltc1865_raw_values[channel])) /
      65536.0f;
  return ltc1865_values_in_V[channel];
}

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle) {

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (spiHandle->Instance == SPI1) {
    /* USER CODE BEGIN SPI1_MspInit 0 */

    /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN SPI1_MspInit 1 */

    /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spiHandle) {

  if (spiHandle->Instance == SPI1) {
    /* USER CODE BEGIN SPI1_MspDeInit 0 */

    /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    /* USER CODE BEGIN SPI1_MspDeInit 1 */

    /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
