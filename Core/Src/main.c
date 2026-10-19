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
#include "can.h"
#include "dac.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sync-demod-api.h"
#include <blinky-api.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Result of one completed lock-in window, handed from the TIM2 ISR to the main loop
struct LockinResult {
    float i;      // In-phase amplitude [ADC counts, peak]
    float q;      // Quadrature amplitude [ADC counts, peak]
    float dc;     // Mean amplifier output [ADC counts]
    uint16_t min; // Smallest sample in the window [ADC counts]
    uint16_t max; // Largest sample in the window [ADC counts]
    bool spi_error;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DAC_MAX 4095
#define VDDA_V 3.3f

// Eccitazione del ponte: sinusoide sul DAC1_OUT1 (PA4). 125 Hz non e' multiplo di 50 Hz,
// e una finestra di 25 periodi (200 ms) contiene anche 10 periodi esatti di rete.
#define EXC_FREQ_HZ 125U
#define EXC_SAMPLES 64U                         // campioni DAC/ADC per periodo
#define TICK_HZ (EXC_FREQ_HZ * EXC_SAMPLES)     // 8 kHz, frequenza dell'ISR di TIM2
#define WINDOW_PERIODS 25U                      // 25 periodi = 200 ms -> 5 misure/s

// Livelli del DAC: 0.25..0.85 V. Con REF dell'amplificatore a ~0.95 V e guadagno 100
// l'uscita resta nel range dell'ADC anche con il ponte sbilanciato di circa +-15 ohm.
#define EXC_DC_V 0.55f
#define EXC_AMP_V 0.30f

// Ponte e amplificatore (schema: R8 = 350R sopra l'estensimetro, ingresso - sul nodo SG_A)
#define AMP_GAIN 100.0f
#define BRIDGE_R_TOP 350.0f

// Calibrazione a due punti: con CAL_ENABLED 0 la resistenza e' stimata dai valori nominali.
// Per calibrare: montare al posto dell'estensimetro le resistenze CAL_R_LO e CAL_R_HI,
// leggere per ciascuna v_out in mV (read_can.py / plot_can.py) e scriverlo in CAL_X_LO_MV / CAL_X_HI_MV.
#define CAL_ENABLED 0
#define CAL_R_LO 348.0f
#define CAL_X_LO_MV 0.0f
#define CAL_R_HI 361.0f
#define CAL_X_HI_MV 0.0f
// Sfasamento della catena analogica (opzionale): phase letta su 0x7A1 con R < 350 ohm
#define CAL_PHASE_DEG 0.0f

// Soglie di saturazione dell'uscita dell'amplificatore
#define CLIP_LOW_V 0.02f
#define CLIP_HIGH_V 3.25f

// 0x7A0/0x7A1: liberi sulla primary (0x19A/0x19B sono DasShutdown/ShutdownState)
#define CAN_ID_LOCKIN 0x7A0U
#define CAN_ID_BRIDGE 0x7A1U

#define FLAG_CLIP_LOW (1U << 0)
#define FLAG_CLIP_HIGH (1U << 1)
#define FLAG_SPI_ERROR (1U << 2)
#define FLAG_UNCALIBRATED (1U << 3)
#define FLAG_NO_DATA (1U << 4)

// Senza misure per questo tempo il main loop invia comunque un frame di stato
#define NO_DATA_TIMEOUT_MS 1000U

#define BLINKY_PATTERN_SIZE (2U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int16_t sin_q15[EXC_SAMPLES];
static int16_t cos_q15[EXC_SAMPLES];
static uint16_t dac_table[EXC_SAMPLES];
static struct SyncDemodHandler demod;
static struct Blinky blinker;

static uint16_t pattern[BLINKY_PATTERN_SIZE] = { 500, 500 };

// Scritti dall'ISR di TIM2, letti dal main loop
static struct LockinResult lockin_result;
static volatile bool lockin_ready = false;
static volatile uint32_t spi_error_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void VectorBase_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Byte di selezione canale LTC1865L (single-ended): bit15=SGL/DIFF=1, bit14=ODD/SIGN.
// CH0 -> ODD/SIGN=0 (0xAA), CH1 -> ODD/SIGN=1 (0xFF). I don't-care ripetono il pattern.
static const uint8_t LTC1865L_CH0[2] = { 0xAA, 0xAA };  // CH1 single-ended = { 0xFF, 0xFF }
#define LTC1865L_VREF 3.29f
#define LTC1865L_FULL_SCALE 65536.0f
#define LTC1865L_LSB_V (LTC1865L_VREF / LTC1865L_FULL_SCALE)
#define LTC1865L_CONV_US 10U     // CONV alto per la conversione (tCONV max di qualche us)
#define SPI_TIMEOUT_US 100U

// Contatore di cicli del core: serve per attese di pochi us dentro l'ISR, dove
// HAL_GetTick() e' fermo (SysTick ha priorita' piu' bassa di TIM2).
static void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t us_to_cycles(uint32_t us) {
    return us * (SystemCoreClock / 1000000U);
}

static void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us_to_cycles(us);
    while (DWT->CYCCNT - start < cycles) {
    }
}

// Scambia un byte su SPI1 (8 bit, FRXTH impostato da HAL_SPI_Init), con timeout.
static bool spi1_xfer(uint8_t tx, uint8_t *rx) {
    uint32_t start = DWT->CYCCNT;
    uint32_t timeout = us_to_cycles(SPI_TIMEOUT_US);

    while ((SPI1->SR & SPI_SR_TXE) == 0U) {
        if (DWT->CYCCNT - start > timeout) {
            return false;
        }
    }
    *(volatile uint8_t *)&SPI1->DR = tx;

    while ((SPI1->SR & SPI_SR_RXNE) == 0U) {
        if (DWT->CYCCNT - start > timeout) {
            return false;
        }
    }
    *rx = *(volatile uint8_t *)&SPI1->DR;
    return true;
}

// Un campione dall'LTC1865L in un solo frame: CONV alto avvia la conversione,
// poi CONV basso e 16 clock che leggono il risultato e caricano la selezione
// canale per la conversione successiva. A regime si legge sempre CH0; il primo
// campione dopo l'avvio puo' essere di un altro canale (scartato con la prima finestra).
static bool ltc1865l_sample(uint16_t *out) {
    while ((SPI1->SR & SPI_SR_RXNE) != 0U) {
        (void)*(volatile uint8_t *)&SPI1->DR; // svuota eventuali residui di un frame fallito
    }

    HAL_GPIO_WritePin(LTC1865L_CS_Port_GPIO_Port, LTC1865L_CS_Port_Pin, GPIO_PIN_SET);
    delay_us(LTC1865L_CONV_US);
    HAL_GPIO_WritePin(LTC1865L_CS_Port_GPIO_Port, LTC1865L_CS_Port_Pin, GPIO_PIN_RESET);

    uint8_t hi = 0;
    uint8_t lo = 0;
    if (!spi1_xfer(LTC1865L_CH0[0], &hi) || !spi1_xfer(LTC1865L_CH0[1], &lo)) {
        return false;
    }
    *out = (uint16_t)(((uint16_t)hi << 8) | lo);
    return true;
}

// Tabelle sin/cos per il demodulatore e tabella del DAC ricavata dalla stessa sin
static void excitation_init(void) {
    sync_demod_api_make_tables(sin_q15, cos_q15, EXC_SAMPLES);
    for (uint32_t k = 0; k < EXC_SAMPLES; k++) {
        float v = EXC_DC_V + EXC_AMP_V * (float)sin_q15[k] / (float)SYNC_DEMOD_Q15_ONE;
        dac_table[k] = (uint16_t)lroundf(v / VDDA_V * (float)DAC_MAX);
    }
}

#if CAL_ENABLED
// Sbilanciamento del ponte d = (V+ - V-) / Vexc per un estensimetro da r ohm
static float bridge_imbalance(float r) {
    return 0.5F - r / (BRIDGE_R_TOP + r);
}
#endif

// Inverte la catena: ampiezza in fase all'uscita dell'amplificatore [V] -> resistenza [ohm].
// Modello: x = k*d + x0 (k = guadagno * ampiezza di eccitazione a meno della calibrazione).
static float bridge_resistance(float x_v) {
#if CAL_ENABLED
    float d_lo = bridge_imbalance(CAL_R_LO);
    float d_hi = bridge_imbalance(CAL_R_HI);
    float x_lo = CAL_X_LO_MV / 1000.0F;
    float x_hi = CAL_X_HI_MV / 1000.0F;
    float k = (x_hi - x_lo) / (d_hi - d_lo);
    float x0 = x_lo - k * d_lo;
#else
    float k = AMP_GAIN * EXC_AMP_V;
    float x0 = 0.0F;
#endif
    float d = (x_v - x0) / k;
    return BRIDGE_R_TOP * (0.5F - d) / (0.5F + d);
}

static void can_send(uint32_t id, const uint8_t data[8]) {
    CAN_TxHeaderTypeDef hdr = { 0 };
    hdr.StdId = id;
    hdr.IDE = CAN_ID_STD;
    hdr.RTR = CAN_RTR_DATA;
    hdr.DLC = 8;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0U) {
        uint32_t mailbox;
        HAL_CAN_AddTxMessage(&hcan, &hdr, (uint8_t *)data, &mailbox);
    }
}

// Converte il risultato di una finestra e lo invia su CAN (formati in acquisinator.dbc):
//   0x7A0 ACQUISINATOR_DIAG:   lockin_out f32 [V], adc_raw u16 (media), adc_poll u8
//   0x7A1 ACQUISINATOR_BRIDGE: resistance f32 [ohm], phase i16 [0.01 deg], flags u8, seq u8
static void lockin_publish(const struct LockinResult *res) {
    static uint8_t seq = 0;

    float i_v = res->i * LTC1865L_LSB_V;
    float q_v = res->q * LTC1865L_LSB_V;
    float cal_phase = CAL_PHASE_DEG * (float)M_PI / 180.0F;
    float x_v = i_v * cosf(cal_phase) + q_v * sinf(cal_phase); // componente in fase, con segno
    float phase_deg = atan2f(q_v, i_v) * 180.0F / (float)M_PI;
    float r_ohm = bridge_resistance(x_v);

    uint8_t flags = 0;
    if ((float)res->min * LTC1865L_LSB_V < CLIP_LOW_V) {
        flags |= FLAG_CLIP_LOW;
    }
    if ((float)res->max * LTC1865L_LSB_V > CLIP_HIGH_V) {
        flags |= FLAG_CLIP_HIGH;
    }
    if (res->spi_error) {
        flags |= FLAG_SPI_ERROR;
    }
#if !CAL_ENABLED
    flags |= FLAG_UNCALIBRATED;
#endif

    uint8_t data[8] = { 0 };
    uint16_t dc = (uint16_t)lroundf(res->dc);
    memcpy(&data[0], &x_v, sizeof(x_v));
    data[4] = (uint8_t)dc;
    data[5] = (uint8_t)(dc >> 8);
    data[6] = (uint8_t)(res->spi_error ? HAL_ERROR : HAL_OK);
    can_send(CAN_ID_LOCKIN, data);

    int16_t phase_cdeg = (int16_t)lroundf(phase_deg * 100.0F);
    memset(data, 0, sizeof(data));
    memcpy(&data[0], &r_ohm, sizeof(r_ohm));
    data[4] = (uint8_t)phase_cdeg;
    data[5] = (uint8_t)((uint16_t)phase_cdeg >> 8);
    data[6] = flags;
    data[7] = seq++;
    can_send(CAN_ID_BRIDGE, data);
}

// Nessuna finestra completata: resistenza NaN e il motivo nei flag, cosi' sul CAN
// si vede che la scheda e' viva anche quando l'ADC non risponde.
static void lockin_publish_no_data(bool spi_error) {
    float r_ohm = NAN;
    uint8_t data[8] = { 0 };
    memcpy(&data[0], &r_ohm, sizeof(r_ohm));
    data[6] = (uint8_t)(FLAG_NO_DATA | (spi_error ? FLAG_SPI_ERROR : 0U));
    can_send(CAN_ID_BRIDGE, data);
}
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
  MX_TIM2_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
    dwt_init();
    excitation_init();
    sync_demod_api_init(&demod, sin_q15, cos_q15, EXC_SAMPLES, WINDOW_PERIODS);
    blinky_api_init(&blinker, pattern, BLINKY_PATTERN_SIZE, true, BLINKY_LOW);
    HAL_CAN_Start(&hcan);

    __HAL_SPI_ENABLE(&hspi1);

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_table[0]);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

    // TIM2 a TICK_HZ: il suo clock e' PCLK1 x2 perche' il prescaler APB1 e' /2
    __HAL_TIM_SET_AUTORELOAD(&htim2, (2U * HAL_RCC_GetPCLK1Freq()) / TICK_HZ - 1U);
    HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    uint32_t last_publish = HAL_GetTick();
    uint32_t last_spi_errors = 0;
    while (1) {
        if (lockin_ready) {
            __disable_irq();
            struct LockinResult res = lockin_result;
            lockin_ready = false;
            __enable_irq();

            lockin_publish(&res);
            last_publish = HAL_GetTick();
            last_spi_errors = spi_error_count;
        } else if (HAL_GetTick() - last_publish >= NO_DATA_TIMEOUT_MS) {
            uint32_t spi_errors = spi_error_count;
            lockin_publish_no_data(spi_errors != last_spi_errors);
            last_publish = HAL_GetTick();
            last_spi_errors = spi_errors;
        }

        enum BlinkyState led_status = blinky_api_routine(&blinker, HAL_GetTick());
        HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, (GPIO_PinState)led_status);
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
// Ogni tick (8 kHz): campiona l'uscita dell'amplificatore, che si e' assestata sul
// valore del DAC scritto al tick precedente, poi avanza il DAC al campione successivo.
// Il campione viene demodulato con l'indice di fase del valore che lo ha prodotto.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    static uint32_t exc_idx = 0;
    static bool window_spi_error = false;
    static bool first_window = true;

    if (htim->Instance != TIM2) {
        return;
    }

    uint16_t raw = 0;
    bool sample_ok = ltc1865l_sample(&raw);
    uint32_t sampled_idx = exc_idx;

    exc_idx = (exc_idx + 1U) % EXC_SAMPLES;
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_table[exc_idx]);

    if (!sample_ok) {
        // Un campione mancante romperebbe i periodi interi: si riparte con la finestra
        sync_demod_api_reset(&demod);
        window_spi_error = true;
        spi_error_count++;
        return;
    }

    if (!sync_demod_api_update(&demod, raw, sampled_idx)) {
        return;
    }

    if (first_window) {
        // La prima finestra contiene il primo campione dell'ADC, non affidabile
        first_window = false;
        window_spi_error = false;
        return;
    }

    lockin_result.i = demod.i;
    lockin_result.q = demod.q;
    lockin_result.dc = demod.dc;
    lockin_result.min = demod.min;
    lockin_result.max = demod.max;
    lockin_result.spi_error = window_spi_error;
    window_spi_error = false;
    lockin_ready = true;
}

static void VectorBase_Config(void) {
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
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
