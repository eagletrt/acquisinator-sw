/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "spi.h"
#include "stm32f3xx_hal_dac.h"
#include "stm32f3xx_hal_dac_ex.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "acquisinator_id.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLES 256
#define DAC_MAX 4095 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#include "demod.h"
//#include <math.h>
#include "lockin.h"
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#define SAMPLING_FREQ   500.0f
#define REF_FREQ        500.0f
#define LPF_CUTOFF      5.0f

#define ADC_FULL_SCALE  4095.0f
#define DAC_FULL_SCALE  4095.0f
#define VCC             3.3f
#define REF_AMPLITUDE   (VCC / 2.0f)

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static uint32_t            print_tick = 0;
static Demod_HandleTypeDef demod;

/* USER CODE BEGIN PV */
static const uint16_t sine_lut[SAMPLES] = {
    2047, 2097, 2147, 2198, 2248, 2298, 2347, 2397, 2446, 2496, 2545, 2593, 2641, 2689, 2737, 2784,
    2831, 2877, 2922, 2968, 3012, 3056, 3100, 3142, 3185, 3226, 3267, 3307, 3346, 3384, 3422, 3459,
    3495, 3530, 3564, 3597, 3630, 3661, 3692, 3721, 3749, 3777, 3803, 3829, 3853, 3876, 3898, 3919,
    3939, 3957, 3975, 3991, 4006, 4020, 4033, 4045, 4055, 4064, 4072, 4079, 4085, 4089, 4092, 4094,
    4095, 4094, 4092, 4089, 4085, 4079, 4072, 4064, 4055, 4045, 4033, 4020, 4006, 3991, 3975, 3957,
    3939, 3919, 3898, 3876, 3853, 3829, 3803, 3777, 3749, 3721, 3692, 3661, 3630, 3597, 3564, 3530,
    3495, 3459, 3422, 3384, 3346, 3307, 3267, 3226, 3185, 3142, 3100, 3056, 3012, 2968, 2922, 2877,
    2831, 2784, 2737, 2689, 2641, 2593, 2545, 2496, 2446, 2397, 2347, 2298, 2248, 2198, 2147, 2097,
    2047, 1997, 1947, 1896, 1846, 1796, 1747, 1697, 1648, 1598, 1549, 1501, 1453, 1405, 1357, 1310,
    1263, 1217, 1172, 1126, 1082, 1038,  994,  952,  909,  868,  827,  787,  748,  710,  672,  635,
     599,  564,  530,  497,  464,  433,  402,  373,  345,  317,  291,  265,  241,  218,  196,  175,
     155,  137,  119,  103,   88,   74,   61,   49,   39,   30,   22,   15,    9,    5,    2,    0,
       0,    0,    2,    5,    9,   15,   22,   30,   39,   49,   61,   74,   88,  103,  119,  137,
     155,  175,  196,  218,  241,  265,  291,  317,  345,  373,  402,  433,  464,  497,  530,  564,
     599,  635,  672,  710,  748,  787,  827,  868,  909,  952,  994, 1038, 1082, 1126, 1172, 1217,
    1263, 1310, 1357, 1405, 1453, 1501, 1549, 1598, 1648, 1697, 1747, 1796, 1846, 1896, 1947, 1997,
};
static volatile uint32_t sine_idx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void VectorBase_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  VectorBase_Config();
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
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
  Demod_Init(&demod, REF_FREQ, LPF_CUTOFF, SAMPLING_FREQ);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* USER CODE END WHILE */
      float signal, reference;

      // Read adc1 from resistor
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, 10);
      signal = (float)HAL_ADC_GetValue(&hadc1) * VCC / ADC_FULL_SCALE;

      reference = HAL_DAC_GetValue(&hdac1, DAC_CHANNEL_1);

      // Update demodulator with new samples
      Demod_Update(&demod, signal, reference);
      
      if (++print_tick >= 100)
      {
          print_tick = 0;
          // H = Y / (A^2 + 0.5) where A is VCC/2
          float v_out = (demod.amplitude) / (((float)VCC/2.0f)*((float)VCC/2.0f) + 0.5f);
          
          HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
          printf("Tensione in uscita: %f V\r\n", v_out);

          printf("-----------------------------------\r\n");
      }

      HAL_Delay(1);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (sine_lut[sine_idx]/1.414f)+600.0f);
    sine_idx = (sine_idx + 1) % SAMPLES;
  }
}

static void VectorBase_Config(void)
{
  /* The constant array with vectors of the vector table is declared externally in the
   * c-startup code.
   */
  extern const unsigned long g_pfnVectors[];
 
  /* Remap the vector table to where the vector table is located for this program. */
  SCB->VTOR = (unsigned long)&g_pfnVectors[0];
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
