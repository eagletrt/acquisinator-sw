/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.h
 * @brief   This file contains all the function prototypes for
 *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#include "secondary_network.h"

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

int can_routine(void);

void acquisinatore_send_version(void);
void acquisinatore_send_errors(void);
void acquisinatore_send_calibration_offsets(float off1, float off2);
void acquisinatore_send_debug_5_values(float v1, float v2, float v3);
void acquisinatore_send_debug_6_values(float v1, float v2, float v3);
void acquisinatore_send_debug_7_values(float v1, float v2, float v3);
void acquisinatore_send_debug_8_values(float v1, float v2, float v3);

void acquisinatore_send_strain_gauge_val_fl_wheel(secondary_link_deformation_fl_wheel_rod_id rod_id, float strain_gauge_val);
void acquisinatore_send_strain_gauge_val_fr_wheel(secondary_link_deformation_fr_wheel_rod_id rod_id, float strain_gauge_val);
void acquisinatore_send_strain_gauge_val_rl_wheel(secondary_link_deformation_rl_wheel_rod_id rod_id, float strain_gauge_val);
void acquisinatore_send_strain_gauge_val_rr_wheel(secondary_link_deformation_rr_wheel_rod_id rod_id, float strain_gauge_val);
void acquisinatore_send_water_cooling_temp(double radiator_input, double radiator_output);
void acquisinatore_send_air_cooling_temp(double air_temperature);
void acquisinatore_rear_send_ammo_pos(float rl, float rr);
void acquisinatore_front_send_ammo_pos(float fl, float fr);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

