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
#include "gpio.h"
#include "spi.h"
#include "tim.h"

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

// TODO: include the crash debug stuff so that we can debug strange stuff remotely
#define CHECK_HAL_RES(res) \
    if (res != HAL_OK) {   \
        return res;        \
    }

HAL_StatusTypeDef save_configs_to_flash(float data1, float data2) {
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

    res = HAL_FLASH_Lock();
    CHECK_HAL_RES(res);
    return HAL_OK;
}

float read_float_from_flash(float *address) {
    if (address == NULL)
        return 0.0f;
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
    return HAL_GetTick();
}

#if ACQUISINATOR_ID == ACQUISINATOR_ID_0
void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    float top_left_ntc_temperature =
        acquisinatore_ntc_from_V_to_degrees_celsius((ch1 / 2.0f) + *o1);  // air temperature of radiator entrance
    float link_deformation = acquisinatore_link_deformation_from_V_to_elongation(ch2 + *o2);

    if (calibration_reset_link_deformation_flag) {
        *o2 = LINK_DEFORMATION_CALIBRATION_VALUE - link_deformation;  // TODO: use the oversampled one
        // save_configs_to_flash(*o1, *o2);
    }
    if ((HAL_GetTick() - *pts) > NTC_COOLING_DELAY_MS) {
        *pts = HAL_GetTick();
        acquisinatore_send_air_cooling_temp(top_left_ntc_temperature);
    }
    acquisinatore_send_strain_gauge_val_rr_wheel(0, link_deformation);
}
#elif ACQUISINATOR_ID == ACQUISINATOR_ID_1
void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    // exit from the radiators (but please check every time)
    float top_right_ntc_temperature = acquisinatore_ntc_from_V_to_degrees_celsius((ch1 / 2.0f) + *o1);
    // entrance of the radiators (but please check every time)
    float bottom_right_ntc_temperature = acquisinatore_ntc_from_V_to_degrees_celsius(((ch2 - 0.8f) / 2.0f) + *o2);
    if ((HAL_GetTick() - *pts) > NTC_COOLING_DELAY_MS) {
        acquisinatore_send_water_cooling_temp(bottom_right_ntc_temperature, top_right_ntc_temperature);
        *pts = HAL_GetTick();
    }
}
#elif ACQUISINATOR_ID == ACQUISINATOR_ID_2
void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    float link_deformation = acquisinatore_link_deformation_from_V_to_elongation(ch2);
    acquisinatore_send_strain_gauge_val_fl_wheel(0, link_deformation);
}
#elif ACQUISINATOR_ID == ACQUISINATOR_ID_3
void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    float link_deformation = acquisinatore_link_deformation_from_V_to_elongation(ch2);
    acquisinatore_send_strain_gauge_val_fr_wheel(0, link_deformation);
}
#elif ACQUISINATOR_ID == ACQUISINATOR_ID_4
void acquisinator_task(float ch1, float ch2, float *o1, float *o2, uint32_t *pts) {
    float link_deformation = acquisinatore_link_deformation_from_V_to_elongation(ch2);
    acquisinatore_send_strain_gauge_val_rl_wheel(0, link_deformation);
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
    uint32_t previous_timestamp = HAL_GetTick();
    float offset1, offset2 = 0.0f;

    // if (save_configs_to_flash(1.0f, 1.0f) < HAL_OK) {
        // Error_Handler();
    // }

    offset1 = read_float_from_flash((float *)ACQUISINATOR_CONFIG_RESERVED_ADDRESS);
    offset2 = read_float_from_flash((float *)(ACQUISINATOR_CONFIG_RESERVED_ADDRESS + sizeof(float)));

    uint32_t version_previous_timestamp = HAL_GetTick();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        if ((HAL_GetTick() - version_previous_timestamp) > SECONDARY_ACQUISINATOR_VERSION_CYCLE_TIME_MS) {
            acquisinatore_send_version();
            version_previous_timestamp = HAL_GetTick();
        }
        float ltc1865_channel1_value_in_V = ltc1865_read(ltc1865_SE_CH1);  // normally for the ntc temperature sensor
        float ltc1865_channel2_value_in_V = ltc1865_read(ltc1865_SE_CH2);  // normally for the strain gauge
        if (ltc1865_channel1_value_in_V == -1 || ltc1865_channel2_value_in_V == -1) {
            acquisinatore_set_led_code(acquisinatore_led_code_spi_error);
        }
        // acquisinatore_send_debug_1_values(offset1, offset2, 0.0f, 0.0f);
        acquisinator_task(ltc1865_channel1_value_in_V, ltc1865_channel2_value_in_V, &offset1, &offset2, &previous_timestamp);
        acquisinatore_set_led_code(last_set_error);
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
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
