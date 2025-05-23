/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "dac.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "acquisinator_config.h"
#include "error_codes.h"
#include "math.h"

#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#if ACQUISINATOR_ID == 0
#error INVALID ACQUISINATOR_ID == 0
#endif

#if ACQUISINATOR_ID == ACQUISINATOR_ID_0
#error INVALID ACQUISINATOR_ID == ACQUISINATOR_ID_0
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

extern uint32_t last_set_error;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

bool calibration_ammo_pos_flag               = false;
bool calibration_reset_link_deformation_flag = false;
uint32_t acquisinator_id_from_flash          = ACQUISINATOR_ID;

#define POT_REST_POS_IDX_DX 0
#define POT_REST_POS_IDX_SX 1
float pot_rest_pos[2] = {DEF_POT_DX_REST_POS, DEF_POT_SX_REST_POS};
// float c_ammo_pos_dx   = 0.0f;
// float c_ammo_pos_sx   = 0.0f;

// TODO: include the crash debug stuff so that we can debug strange stuff remotely
#define CHECK_HAL_RES(res) \
    if (res != HAL_OK) {   \
        return res;        \
    }

HAL_StatusTypeDef save_configs_to_flash(float data1, float data2, uint32_t acquisinatore_version) {
    HAL_StatusTypeDef res = HAL_FLASH_Unlock();
    CHECK_HAL_RES(res);

    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase = FLASH_TYPEERASE_PAGES, .PageAddress = ACQUISINATOR_CONFIG_RESERVED_ADDRESS, .NbPages = 1};
    uint32_t mem_error = 0;
    res                = HAL_FLASHEx_Erase(&erase_init, &mem_error);
    CHECK_HAL_RES(res);

    union {
        uint32_t d;
        float f;
    } flash_config_union = {.f = data1};

    res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ACQUISINATOR_CONFIG_RESERVED_ADDRESS, flash_config_union.d);
    CHECK_HAL_RES(res);

    flash_config_union.f = data2;
    res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ACQUISINATOR_CONFIG_RESERVED_ADDRESS + sizeof(data1), flash_config_union.d);
    CHECK_HAL_RES(res);

    res = HAL_FLASH_Program(
        FLASH_TYPEPROGRAM_WORD, ACQUISINATOR_CONFIG_RESERVED_ADDRESS + sizeof(data1) + sizeof(data2), acquisinatore_version);
    CHECK_HAL_RES(res);

    res = HAL_FLASH_Lock();
    CHECK_HAL_RES(res);
    return HAL_OK;
}

float read_float_from_flash(float *address) {
    if (address == NULL)
        return 0.0f;
    return *address;
}

uint32_t read_uint32_from_flash(uint32_t *address) {
    if (address == NULL)
        return 0U;
    return *address;
}

float acquisinatore_ntc_from_V_to_degrees_celsius(float value_in_V) {
    return (
        144.753928685635f + (-441.361483066664f * value_in_V) + (808.342944901787f * powf(value_in_V, 2.0f)) +
        (-831.830860786940f * powf(value_in_V, 3.0f)) + (425.004221588082f * powf(value_in_V, 4.0f)) +
        (-84.5939048598991f * powf(value_in_V, 5.0f)));
}

float acquisinatore_link_deformation_from_V_to_elongation(float value_in_V) {
    return (
        (STRAIN_GAUGE_RNOM -
         ((STRAIN_GAUGE_R2 * STRAIN_GAUGE_R3) -
          (STRAIN_GAUGE_R2 * ((value_in_V - STRAIN_GAUGE_VREF) / STRAIN_GAUGE_G) * (STRAIN_GAUGE_R1 + STRAIN_GAUGE_R2)) / STRAIN_GAUGE_VS) /
             (STRAIN_GAUGE_R1 +
              ((((value_in_V - STRAIN_GAUGE_VREF) / STRAIN_GAUGE_G) * (STRAIN_GAUGE_R1 + STRAIN_GAUGE_R2)) / STRAIN_GAUGE_VS))) /
        (STRAIN_GAUGE_RNOM * STRAIN_GAUGE_K));
}

void acquisinatore_turn_led(int on) {
    if (on) {
        HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);
    }
}

uint32_t get_timestamp_ms(void) {
    return HAL_GetTick();  //+ ACQUISINATOR_ID;
}

void system_reset(void) {
    HAL_NVIC_SystemReset();
}

// This assumes that o2 is always the offset for strain gauge
HAL_StatusTypeDef link_deformation_calibration_check(float ch, float *o1, float *o2) {
    if (calibration_reset_link_deformation_flag) {
        calibration_reset_link_deformation_flag = false;
        *o2                                     = LINK_DEFORMATION_CALIBRATION_VALUE - ch;
        return save_configs_to_flash(*o1, *o2, acquisinator_id_from_flash);
    }
    return HAL_OK;
}

HAL_StatusTypeDef ammo_compression_calibration_check(float c_ammo_pos_dx, float c_ammo_pos_sx, float *o1, float *o2) {
    if (calibration_ammo_pos_flag) {
        calibration_ammo_pos_flag = false;
        *o1                       = -c_ammo_pos_dx;
        *o2                       = -c_ammo_pos_sx;
        return save_configs_to_flash(*o1, *o2, acquisinator_id_from_flash);
    }
    return HAL_OK;
}

float link_deformation_routine(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    float link_deformation = acquisinatore_link_deformation_from_V_to_elongation(ch2 + *o2);
    if (link_deformation_calibration_check(ch2, o1, o2) != HAL_OK) {
        acquisinatore_set_led_code(acquisinatore_led_code_read_write_flash);
    }
#if ACQUISINATORE_SEND_CALIBRATIONS_OFFSETS
    static uint32_t calibration_values_last_send = 0;
    if ((get_timestamp_ms() + acquisinator_id_from_flash - calibration_values_last_send) >
        SECONDARY_ACQUISINATOR_CALIBRATIONS_OFFSETS_CYCLE_TIME_MS) {
        calibration_values_last_send = get_timestamp_ms();
        acquisinatore_send_calibration_offsets(*o1, *o2);
    }
#endif
    return link_deformation;
}

#if ACQUISINATOR_ID == ACQUISINATOR_ID_5

// rear ammos
void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    static uint32_t last_send = 0;
    ch2 -= OFFSET_FOR_SECOND_CHANNEL_mV;

    float pot_pos_dx    = FROM_MILLIVOLT_TO_POT_POS(ch1 * 1000.0f);
    float pot_pos_sx    = FROM_MILLIVOLT_TO_POT_POS(ch2 * 1000.0f);
    float c_ammo_pos_dx = POT_TO_AMMO_POS(pot_pos_dx) * (-1.0f) + *o1;
    float c_ammo_pos_sx = POT_TO_AMMO_POS(pot_pos_sx) * (-1.0f) + *o2;

    if (ammo_compression_calibration_check(c_ammo_pos_dx, c_ammo_pos_sx, o1, o2) != HAL_OK) {
        // TODO: handle errors
    }

    if (get_timestamp_ms() - last_send > ACQUISINATORE_AMMO_POSITION_DELAY_MS) {
        last_send = get_timestamp_ms();
        acquisinatore_send_debug_7_values(ch1 / 10.0f, ch2 / 10.0f, 0.0f);
        acquisinatore_rear_send_ammo_pos(c_ammo_pos_sx, c_ammo_pos_dx);
    }
}

#elif ACQUISINATOR_ID == ACQUISINATOR_ID_8

// front ammos
void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    static uint32_t last_send = 0;
    ch2 -= OFFSET_FOR_SECOND_CHANNEL_mV;

    float pot_pos_dx    = FROM_MILLIVOLT_TO_POT_POS(ch1 * 1000.0f);
    float pot_pos_sx    = FROM_MILLIVOLT_TO_POT_POS(ch2 * 1000.0f);
    float c_ammo_pos_dx = POT_TO_AMMO_POS(pot_pos_dx) * (-1.0f) + *o1;
    float c_ammo_pos_sx = POT_TO_AMMO_POS(pot_pos_sx) * (-1.0f) + *o2;

    if (ammo_compression_calibration_check(c_ammo_pos_dx, c_ammo_pos_sx, o1, o2) != HAL_OK) {
        // TODO: handle errors
    }
    if (get_timestamp_ms() - last_send > ACQUISINATORE_AMMO_POSITION_DELAY_MS) {
        last_send = get_timestamp_ms();
        acquisinatore_send_debug_6_values(ch1 / 10.0f, ch2 / 10.0f, 0.0f);
        acquisinatore_front_send_ammo_pos(c_ammo_pos_sx, c_ammo_pos_dx);
    }
}

#elif ACQUISINATOR_ID == ACQUISINATOR_ID_9

void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    float link_deformation                     = link_deformation_routine(ch1, ch2, o1, o2, pts);
    static uint32_t last_link_deformation_sent = 0;
    if ((get_timestamp_ms() - last_link_deformation_sent) > ACQUISINATORE_LINK_DEFORMATION_DELAY_MS) {
        last_link_deformation_sent = get_timestamp_ms();
        acquisinatore_send_strain_gauge_val_rr_wheel(link_deformation, secondary_link_deformation_rr_wheel_rod_id_F1011);
    }
}

#elif ACQUISINATOR_ID == REAR_RIGHT_WHEEL_F36

void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    float link_deformation                     = link_deformation_routine(ch1, ch2, o1, o2, pts);
    static uint32_t last_link_deformation_sent = 0;
    if ((get_timestamp_ms() - last_link_deformation_sent) > ACQUISINATORE_LINK_DEFORMATION_DELAY_MS) {
        last_link_deformation_sent = get_timestamp_ms();
        acquisinatore_send_strain_gauge_val_rr_wheel(link_deformation, secondary_link_deformation_rr_wheel_rod_id_F36);
    }
}

#elif ACQUISINATOR_ID == REAR_RIGHT_WHEEL_F46

void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    float link_deformation                     = link_deformation_routine(ch1, ch2, o1, o2, pts);
    static uint32_t last_link_deformation_sent = 0;
    if ((get_timestamp_ms() - last_link_deformation_sent) > ACQUISINATORE_LINK_DEFORMATION_DELAY_MS) {
        last_link_deformation_sent = get_timestamp_ms();
        acquisinatore_send_strain_gauge_val_rr_wheel(link_deformation, secondary_link_deformation_rr_wheel_rod_id_F46);
    }
}

#elif ACQUISINATOR_ID == REAR_RIGHT_WHEEL_F17

void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    float link_deformation                     = link_deformation_routine(ch1, ch2, o1, o2, pts);
    static uint32_t last_link_deformation_sent = 0;
    if ((get_timestamp_ms() - last_link_deformation_sent) > ACQUISINATORE_LINK_DEFORMATION_DELAY_MS) {
        last_link_deformation_sent = get_timestamp_ms();
        acquisinatore_send_strain_gauge_val_rr_wheel(link_deformation, secondary_link_deformation_rr_wheel_rod_id_F17);
    }
}

#elif ACQUISINATOR_ID == REAR_RIGHT_WHEEL_F58

void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    float link_deformation                     = link_deformation_routine(ch1, ch2, o1, o2, pts);
    static uint32_t last_link_deformation_sent = 0;
    if ((get_timestamp_ms() - last_link_deformation_sent) > ACQUISINATORE_LINK_DEFORMATION_DELAY_MS) {
        last_link_deformation_sent = get_timestamp_ms();
        acquisinatore_send_strain_gauge_val_rr_wheel(link_deformation, secondary_link_deformation_rr_wheel_rod_id_F58);
    }
}

#elif ACQUISINATOR_ID == REAR_RIGHT_WHEEL_F27

void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    float link_deformation                     = link_deformation_routine(ch1, ch2, o1, o2, pts);
    static uint32_t last_link_deformation_sent = 0;
    if ((get_timestamp_ms() - last_link_deformation_sent) > ACQUISINATORE_LINK_DEFORMATION_DELAY_MS) {
        last_link_deformation_sent = get_timestamp_ms();
        acquisinatore_send_strain_gauge_val_rr_wheel(link_deformation, secondary_link_deformation_rr_wheel_rod_id_F27);
    }
}

#elif ACQUISINATOR_ID == REAR_LEFT_WHEEL_F36

void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    float link_deformation                     = link_deformation_routine(ch1, ch2, o1, o2, pts);
    static uint32_t last_link_deformation_sent = 0;
    if ((get_timestamp_ms() - last_link_deformation_sent) > ACQUISINATORE_LINK_DEFORMATION_DELAY_MS) {
        last_link_deformation_sent = get_timestamp_ms();
        acquisinatore_send_strain_gauge_val_rl_wheel(link_deformation, secondary_link_deformation_rl_wheel_rod_id_F36);
    }
}

#elif ACQUISINATOR_ID == REAR_LEFT_WHEEL_F46

void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    float link_deformation                     = link_deformation_routine(ch1, ch2, o1, o2, pts);
    static uint32_t last_link_deformation_sent = 0;
    if ((get_timestamp_ms() - last_link_deformation_sent) > ACQUISINATORE_LINK_DEFORMATION_DELAY_MS) {
        last_link_deformation_sent = get_timestamp_ms();
        acquisinatore_send_strain_gauge_val_rl_wheel(link_deformation, secondary_link_deformation_rl_wheel_rod_id_F46);
    }
}

#elif ACQUISINATOR_ID == REAR_LEFT_WHEEL_F17

void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    float link_deformation                     = link_deformation_routine(ch1, ch2, o1, o2, pts);
    static uint32_t last_link_deformation_sent = 0;
    if ((get_timestamp_ms() - last_link_deformation_sent) > ACQUISINATORE_LINK_DEFORMATION_DELAY_MS) {
        last_link_deformation_sent = get_timestamp_ms();
        acquisinatore_send_strain_gauge_val_rl_wheel(link_deformation, secondary_link_deformation_rl_wheel_rod_id_F17);
    }
}

#elif ACQUISINATOR_ID == REAR_LEFT_WHEEL_F58

void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    float link_deformation                     = link_deformation_routine(ch1, ch2, o1, o2, pts);
    static uint32_t last_link_deformation_sent = 0;
    if ((get_timestamp_ms() - last_link_deformation_sent) > ACQUISINATORE_LINK_DEFORMATION_DELAY_MS) {
        last_link_deformation_sent = get_timestamp_ms();
        acquisinatore_send_strain_gauge_val_rl_wheel(link_deformation, secondary_link_deformation_rl_wheel_rod_id_F58);
    }
}

#elif ACQUISINATOR_ID == REAR_LEFT_WHEEL_F27

void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    float link_deformation                     = link_deformation_routine(ch1, ch2, o1, o2, pts);
    static uint32_t last_link_deformation_sent = 0;
    if ((get_timestamp_ms() - last_link_deformation_sent) > ACQUISINATORE_LINK_DEFORMATION_DELAY_MS) {
        last_link_deformation_sent = get_timestamp_ms();
        acquisinatore_send_strain_gauge_val_rl_wheel(link_deformation, secondary_link_deformation_rr_wheel_rod_id_F27);
    }
}

#else

#warning EMPTY ACQUISINATOR TASK
void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
}

#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_CAN_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

#if ACQUISINATORE_FILTER_TYPE == ACQUISINATORE_NO_FILTER
#elif ACQUISINATORE_FILTER_TYPE == ACQUISINATORE_SIMPLE_MOVING_AVG

#elif ACQUISINATORE_FILTER_TYPE == ACQUISINATORE_GAUSSIAN_FILTER
    lpfilter_init();
#elif ACQUISINATORE_FILTER_TYPE == ACQUISINATORE_OLD_MOVING_AVG
#endif

    acquisinatore_set_led_code(acquisinatore_led_code_all_ok);
    uint32_t previous_timestamp               = get_timestamp_ms();
    uint32_t version_previous_timestamp       = get_timestamp_ms();
    uint32_t errors_in_can_previous_timestamp = get_timestamp_ms();

    float offset1, offset2 = 0.0f;

#if ACQUISINATOR_RESETS_TO_DEFAULT_CONFIGS == 1
    // RESET CONFIGS TO FLASH
    if (save_configs_to_flash(0.0f, 0.0f, ACQUISINATOR_ID) != HAL_OK) {
        Error_Handler();
    }
#endif

    offset1                    = read_float_from_flash((float *)ACQUISINATOR_CONFIG_RESERVED_ADDRESS);
    offset2                    = read_float_from_flash((float *)(ACQUISINATOR_CONFIG_RESERVED_ADDRESS + sizeof(float)));
    acquisinator_id_from_flash = read_uint32_from_flash((uint32_t *)(ACQUISINATOR_CONFIG_RESERVED_ADDRESS + (2 * sizeof(float))));

    if (acquisinator_id_from_flash == 0 || acquisinator_id_from_flash != ACQUISINATOR_ID) {
        // the wrong firmware was flashed!!!
        acquisinatore_set_led_code(acquisinatore_led_code_flashed_firmware_with_wrong_id);
    }

    /*
    // [not really necessary] save the read config again to flash
    if (save_configs_to_flash(offset1, offset2, acquisinator_id_from_flash) != HAL_OK) {
        Error_Handler();
    }
    */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
        if (((get_timestamp_ms() + acquisinator_id_from_flash) - version_previous_timestamp) >
            SECONDARY_ACQUISINATOR_VERSION_CYCLE_TIME_MS) {
            acquisinatore_send_version();
            version_previous_timestamp = get_timestamp_ms();
        }
        if (((get_timestamp_ms() + acquisinator_id_from_flash) - errors_in_can_previous_timestamp) >
            SECONDARY_ACQUISINATOR_ERRORS_CYCLE_TIME_MS) {
            acquisinatore_send_errors();
            errors_in_can_previous_timestamp = get_timestamp_ms();
        }
        float ltc1865_channel1_value_in_V = ltc1865_read(ltc1865_SE_CH1);  // normally the channel for the ntc temperature sensor
        float ltc1865_channel2_value_in_V = ltc1865_read(ltc1865_SE_CH2);  // normally the channel for the strain gauge
        if (ltc1865_channel1_value_in_V == -1 || ltc1865_channel2_value_in_V == -1) {
            acquisinatore_set_led_code(acquisinatore_led_code_spi_error);
        }
        acquisinator_task(ltc1865_channel1_value_in_V, ltc1865_channel2_value_in_V, &offset1, &offset2, &previous_timestamp);
        acquisinatore_led_code_routine();
        can_routine();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    HAL_NVIC_SystemReset();
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
