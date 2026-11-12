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
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "bridge-api.h"
#include "exc-runner-api.h"
#include <blinky-api.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DAC_MAX 4095
#define VDDA_V 3.3f
// Escursione dell'uscita con il buffer del DAC attivo (datasheet F334, "DAC characteristics":
// da ricontrollare). Il buffer e' specificato per carichi >= 5 kohm, il ponte ne presenta
// ~350: l'ampiezza reale sul ponte e' piu' bassa di quella impostata, va misurata su PA4.
#define DAC_OUT_MIN_V 0.2f
#define DAC_OUT_MAX_V (VDDA_V - 0.2f)

// Eccitazione del ponte: sinusoide sul DAC1_OUT1 (PA4). Frequenza e livelli si cambiano a
// runtime con il frame CAN 0x7A2; la temporizzazione la calcola exc_plan_api_make.
// All'avvio: 125 Hz, 0.25..0.85 V. Con REF dell'amplificatore a ~0.95 V e guadagno 100
// l'uscita resta nel range dell'ADC anche con il ponte sbilanciato di circa +-15 ohm.
#define EXC_FREQ_DEFAULT_HZ 125U
#define EXC_DC_DEFAULT_MV 550U
#define EXC_AMP_DEFAULT_MV 300U
#define EXC_MAX_SAMPLES 64U   // campioni DAC/ADC per periodo, finche' il tick lo permette
#define EXC_MIN_SAMPLES 16U   // sotto non si accetta: sopra EXC_MAX_TICK_HZ / 16 = 1 kHz
// Tetto del tick di TIM2: l'ISR dura ~30 us (conversione 10 us + 16 bit SPI a 1.125 MHz)
#define EXC_MAX_TICK_HZ 16000U

// ---- le impostazioni da cambiare ----
// Finestra minima del demodulatore: si usa la piu' corta >= WINDOW_MS fatta di periodi interi
// di eccitazione che sia anche un numero intero di periodi di rete (entro MAINS_TOL_HZ):
// cosi' 50 Hz e armoniche si cancellano. A 125 Hz con 40 ms: 5 periodi, esatti.
//   40 ms: banda ~11 Hz (prove dinamiche); 200 ms: banda ~2 Hz, rumore ~2.2 volte piu' basso
#define WINDOW_MS 40U
#define WINDOW_MAX_MS 400U   // oltre, la frequenza viene rifiutata
#define MAINS_TOL_HZ 0.1f    // la rete stessa deriva di ~0.1-0.2 Hz: oltre non serve
// Distanza minima da ogni armonica di rete. Con la rete spostata di d Hz l'armonica k entra
// nella misura per circa k * d / distanza della sua ampiezza: a 125 Hz (25 Hz da 100 e 150)
// lo 0.4% con d = 0.1 Hz, a 5 Hz di distanza il 2%. In macchina la rete non c'e'.
#define MAINS_GUARD_HZ 5.0f
// Ogni quanto inviare una misura (finestra mobile), arrotondato a periodi interi:
// 8 ms = 125 misure/s, 2 frame CAN l'una (0x7A0 + 0x7A1) = 250 frame/s, ~3% di 1 Mbit/s.
#define OUTPUT_EVERY_MS 8U
// -------------------------------------

#if EXC_MAX_SAMPLES > EXC_RUNNER_MAX_SAMPLES
#error "EXC_MAX_SAMPLES oltre le tabelle di ExcSetting"
#endif
#if WINDOW_MS == 0U || WINDOW_MS > WINDOW_MAX_MS || OUTPUT_EVERY_MS == 0U
#error "serve 0 < WINDOW_MS <= WINDOW_MAX_MS e OUTPUT_EVERY_MS almeno 1"
#endif

// Amplificatore: guadagno differenziale (AD8553). L'uscita del ponte e' in mV/V: in fase al
// ponte diviso l'ampiezza di eccitazione, proporzionale alla forza; la forza si calibra su questa.
#define AMP_GAIN 100.0f
// Sfasamento della catena analogica (opzionale): phase letta su 0x7A1 con il ponte sbilanciato
#define CAL_PHASE_DEG 0.0f

// Soglie di saturazione dell'uscita dell'amplificatore
#define CLIP_LOW_V 0.02f
#define CLIP_HIGH_V 3.25f

// 0x7A0..0x7A5: liberi sulla primary (0x19A/0x19B sono DasShutdown/ShutdownState)
#define CAN_ID_LOCKIN 0x7A0U
#define CAN_ID_BRIDGE 0x7A1U
#define CAN_ID_SET_EXC 0x7A2U  // ricevuto: freq u16 [Hz], amp u16 [mV], dc u16 [mV]; 0 = invariato
#define CAN_ID_EXC 0x7A3U      // eccitazione in uso: dopo ogni comando e ogni EXC_STATUS_MS
#define CAN_ID_STREAM 0x7A4U   // campioni grezzi dell'ADC, solo con lo streaming acceso
#define CAN_ID_SET_STREAM 0x7A5U // ricevuto: enable u8 (1 = acceso per STREAM_KEEPALIVE_MS)

// Streaming dei campioni grezzi: l'ISR li accoda, il main loop li spedisce a STREAM_PER_FRAME per
// frame. Si spegne da solo se il comando non viene rinnovato: sul bus della macchina e' ~1/3 del traffico.
#define STREAM_RING_SIZE 1024U // potenza di 2: ~128 ms a 8 kHz
#define STREAM_PER_FRAME 3U
#define STREAM_KEEPALIVE_MS 3000U
// Etichetta di ogni campione nella coda e nel frame (byte phase): indice di fase nei bit 0..5 e
#define STREAM_TAG_CH1 0x40U  // campione di CH1 (altrimenti CH0)
#define STREAM_TAG_DUAL 0x80U // streaming di CH0 e CH1 alternati, uno per canale a ogni tick

// Con -DACQ_STREAM_CH1 (build_flags in platformio.ini), solo a streaming acceso, a ogni tick si converte
// anche CH1 subito dopo CH0, per avere a banco l'eccitazione vera (es. da un generatore esterno).
// Lock-in e frame di misura restano quelli di sempre, su CH0. Senza il flag nulla cambia.

// Senza misure per questo tempo il main loop invia comunque un frame di stato
#define NO_DATA_TIMEOUT_MS 1000U
#define EXC_STATUS_MS 1000U

#define BLINKY_PATTERN_SIZE (2U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Eccitazione e acquisizione: due impostazioni, l'ISR gira su una mentre il main loop
// prepara l'altra (exc_runner_api_request), adottata a fine periodo.
static struct ExcRunnerConfig exc_cfg = {
    .plan = {
        .max_samples = EXC_MAX_SAMPLES,
        .min_samples = EXC_MIN_SAMPLES,
        .max_tick_hz = EXC_MAX_TICK_HZ,
        .max_periods = SYNC_DEMOD_MAX_PERIODS,
        .min_window_ms = WINDOW_MS,
        .max_window_ms = WINDOW_MAX_MS,
        .output_ms = OUTPUT_EVERY_MS,
        .mains_hz = 50.0F,
        .mains_tol_hz = MAINS_TOL_HZ,
        .mains_guard_hz = MAINS_GUARD_HZ,
    },
    .vdda_v = VDDA_V,
    .out_min_v = DAC_OUT_MIN_V,
    .out_max_v = DAC_OUT_MAX_V,
    .dac_max = DAC_MAX,
    .default_freq_hz = EXC_FREQ_DEFAULT_HZ,
    .default_dc_mv = EXC_DC_DEFAULT_MV,
    .default_amp_mv = EXC_AMP_DEFAULT_MV,
};
static struct ExcRunner runner;
static struct Blinky blinker;

static uint16_t pattern[BLINKY_PATTERN_SIZE] = { 500, 500 };

// Scritti dall'ISR di TIM2, letti dal main loop
static struct LockinResult lockin_result;
static volatile bool lockin_ready = false;
static volatile uint32_t spi_error_count = 0;
static volatile uint32_t overrun_count = 0;

// Coda dello streaming: un solo produttore (ISR) e un solo consumatore (main loop)
static uint32_t stream_ring[STREAM_RING_SIZE]; // campione | etichetta << 16
static volatile uint32_t stream_head = 0;      // scritto dall'ISR
static volatile uint32_t stream_tail = 0;      // scritto dal main loop
static volatile bool stream_on = false;
static uint32_t stream_until = 0;
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
static const uint8_t LTC1865L_CH0[2] = { 0xAA, 0xAA };
#ifdef ACQ_STREAM_CH1
static const uint8_t LTC1865L_CH1[2] = { 0xFF, 0xFF };
#endif
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
// canale `next` per la conversione successiva. A regime il primo campione di ogni tick e'
// sempre CH0; il primo campione dopo l'avvio puo' essere di un altro canale (scartato con la
// prima finestra).
static bool ltc1865l_sample(const uint8_t next[2], uint16_t *out) {
    while ((SPI1->SR & SPI_SR_RXNE) != 0U) {
        (void)*(volatile uint8_t *)&SPI1->DR; // svuota eventuali residui di un frame fallito
    }

    HAL_GPIO_WritePin(LTC1865L_CS_Port_GPIO_Port, LTC1865L_CS_Port_Pin, GPIO_PIN_SET);
    delay_us(LTC1865L_CONV_US);
    HAL_GPIO_WritePin(LTC1865L_CS_Port_GPIO_Port, LTC1865L_CS_Port_Pin, GPIO_PIN_RESET);

    uint8_t hi = 0;
    uint8_t lo = 0;
    if (!spi1_xfer(next[0], &hi) || !spi1_xfer(next[1], &lo)) {
        return false;
    }
    *out = (uint16_t)(((uint16_t)hi << 8) | lo);
    return true;
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

// Impostazione di default, prima di avviare TIM2
static void exc_init(void) {
    exc_cfg.plan.tim_clk_hz = 2U * HAL_RCC_GetPCLK1Freq(); // PCLK1 x2 perche' il prescaler APB1 e' /2
    if (!exc_runner_api_init(&runner, &exc_cfg)) {
        Error_Handler();
    }
}

static void exc_publish(const struct ExcSetting *exc) {
    uint8_t data[8];
    exc_runner_api_pack_status(exc, data);
    can_send(CAN_ID_EXC, data);
}

// Accetta solo i frame 0x7A2 e 0x7A5 nella FIFO0
static void can_rx_filter_init(void) {
    CAN_FilterTypeDef filter = { 0 };
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = CAN_ID_SET_EXC << 5U;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = CAN_ID_SET_STREAM << 5U; // in IDLIST e' il secondo id
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
        Error_Handler();
    }
}

static void stream_command(bool enable) {
    stream_until = HAL_GetTick() + STREAM_KEEPALIVE_MS;
    stream_on = enable;
}

// Svuota la FIFO0: 0x7A5 accende o spegne lo streaming, ogni 0x7A2 diventa una richiesta per l'ISR. Se rifiutata si risponde
// subito con l'impostazione in uso (id invariato), se accettata dopo che l'ISR l'ha applicata.
static void can_rx_poll(void) {
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8] = { 0 };
    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &hdr, data) != HAL_OK) {
            return;
        }
        if (hdr.IDE != CAN_ID_STD || hdr.RTR != CAN_RTR_DATA) {
            continue;
        }
        if (hdr.StdId == CAN_ID_SET_STREAM) {
            stream_command(hdr.DLC > 0U && data[0] != 0U);
            continue;
        }
        if (hdr.StdId != CAN_ID_SET_EXC) {
            continue;
        }
        for (uint32_t k = hdr.DLC; k < sizeof(data); k++) {
            data[k] = 0; // byte mancanti = invariato
        }
        uint32_t freq_hz = (uint32_t)data[0] | ((uint32_t)data[1] << 8);
        uint32_t amp_mv = (uint32_t)data[2] | ((uint32_t)data[3] << 8);
        uint32_t dc_mv = (uint32_t)data[4] | ((uint32_t)data[5] << 8);
        if (!exc_runner_api_request(&runner, freq_hz, dc_mv, amp_mv)) {
            exc_publish(runner.active);
        }
    }
}

static const struct BridgeConfig bridge_cfg = {
    .amp_gain = AMP_GAIN,
    .adc_lsb_v = LTC1865L_LSB_V,
    .cal_phase_rad = CAL_PHASE_DEG * (float)M_PI / 180.0F,
    .clip_low_v = CLIP_LOW_V,
    .clip_high_v = CLIP_HIGH_V,
};

// Spedisce i campioni accodati, solo con tutte e tre le mailbox libere: le altre due restano ai
// frame di misura e di stato, che hanno anche id piu' bassi e vincono l'arbitraggio.
//   0x7A4 ACQUISINATOR_ADC_STREAM: 3 campioni u16, etichetta u8 del primo, seq u8
// Con la coda piena l'ISR scarta i campioni: il ricevitore lo vede da un salto dell'indice di fase.
static void stream_poll(void) {
    static uint8_t seq = 0;
    if (stream_on && (int32_t)(HAL_GetTick() - stream_until) >= 0) {
        stream_on = false;
    }
    uint32_t tail = stream_tail;
    if (!stream_on) {
        stream_tail = stream_head; // butta quello che resta
        return;
    }
    while (stream_head - tail >= STREAM_PER_FRAME && HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 3U) {
        atomic_signal_fence(memory_order_acquire); // campioni letti dopo stream_head
        uint8_t data[8];
        for (uint32_t k = 0; k < STREAM_PER_FRAME; k++) {
            uint32_t entry = stream_ring[(tail + k) % STREAM_RING_SIZE];
            data[2U * k] = (uint8_t)entry;
            data[2U * k + 1U] = (uint8_t)(entry >> 8);
        }
        data[6] = (uint8_t)(stream_ring[tail % STREAM_RING_SIZE] >> 16);
        data[7] = seq++;
        tail += STREAM_PER_FRAME;
        stream_tail = tail;
        can_send(CAN_ID_STREAM, data);
    }
}

// Converte il risultato di una finestra e lo invia su CAN (formati in acquisinator.dbc):
//   0x7A0 ACQUISINATOR_DIAG:   lockin_out f32 [V], adc_raw u16 (media), adc_poll u8, exc_id u8 (vedi 0x7A3)
//   0x7A1 ACQUISINATOR_BRIDGE: bridge_out f32 [mV/V], phase i16 [0.01 deg], flags u8, seq u8
static void lockin_publish(const struct LockinResult *res) {
    static uint8_t seq = 0;
    struct BridgeReading reading;
    uint8_t data[8];

    bridge_api_convert(&bridge_cfg, res, &reading);
    bridge_api_pack_diag(res, &reading, data);
    can_send(CAN_ID_LOCKIN, data);
    bridge_api_pack_bridge(&reading, seq++, data);
    can_send(CAN_ID_BRIDGE, data);
}

// Nessuna finestra completata: uscita NaN e il motivo nei flag, cosi' sul CAN
// si vede che la scheda e' viva anche quando l'ADC non risponde.
static void lockin_publish_no_data(bool spi_error) {
    uint8_t data[8];
    bridge_api_pack_no_data(spi_error, data);
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
    exc_init();
    blinky_api_init(&blinker, pattern, BLINKY_PATTERN_SIZE, true, BLINKY_LOW);
    can_rx_filter_init();
    HAL_CAN_Start(&hcan);

    __HAL_SPI_ENABLE(&hspi1);

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, runner.active->dac[0]);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

    // TIM2 a f * campioni per periodo. Timer fermo: ARR scritto direttamente, poi il preload fa si'
    // che i cambi dall'ISR entrino all'update successivo (mai ARR < CNT, che bloccherebbe
    // il contatore a 32 bit per ~60 s)
    __HAL_TIM_SET_AUTORELOAD(&htim2, runner.active->plan.autoreload);
    SET_BIT(htim2.Instance->CR1, TIM_CR1_ARPE);
    HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    uint32_t last_publish = HAL_GetTick();
    uint32_t last_spi_errors = 0;
    uint32_t last_exc_status = HAL_GetTick();
    uint8_t exc_reported_id = runner.active->id;
    exc_publish(runner.active);
    while (1) {
        can_rx_poll();

        // nuova impostazione applicata dall'ISR, oppure stato periodico per i log
        const struct ExcSetting *exc = runner.active;
        if (exc->id != exc_reported_id || HAL_GetTick() - last_exc_status >= EXC_STATUS_MS) {
            exc_publish(exc);
            exc_reported_id = exc->id;
            last_exc_status = HAL_GetTick();
        }

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

        stream_poll();

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
// Accoda un campione per lo streaming; con la coda piena lo scarta.
static void stream_push(uint16_t raw, uint32_t tag) {
    uint32_t head = stream_head;
    if (head - stream_tail < STREAM_RING_SIZE) {
        stream_ring[head % STREAM_RING_SIZE] = (uint32_t)raw | (tag << 16U);
        atomic_signal_fence(memory_order_release); // campione scritto prima di stream_head
        stream_head = head + 1U;
    }
}

// Ogni tick (f * campioni per periodo, 8 kHz a 125 Hz): campiona l'uscita dell'amplificatore, che si e' assestata sul
// valore del DAC scritto al tick precedente, poi avanza il DAC al campione successivo.
// Il campione viene demodulato con l'indice di fase del valore che lo ha prodotto.
static void tim2_tick(TIM_HandleTypeDef *htim) {
    uint16_t raw = 0;
#ifdef ACQ_STREAM_CH1
    // letto una volta: selezione e campioni dello stesso tick restano coerenti; la conversione di CH1
    // riseleziona CH0, quindi il primo campione del tick dopo e' sempre CH0
    bool dual = stream_on;
    bool sample_ok = ltc1865l_sample(dual ? LTC1865L_CH1 : LTC1865L_CH0, &raw);
    uint16_t raw1 = 0;
    bool sample1_ok = dual && ltc1865l_sample(LTC1865L_CH0, &raw1);
#else
    bool sample_ok = ltc1865l_sample(LTC1865L_CH0, &raw);
#endif

    struct ExcStep step;
    exc_runner_api_advance(&runner, &step);
    if (step.switched) {
        // con il preload il nuovo periodo entra all'update successivo
        __HAL_TIM_SET_AUTORELOAD(htim, step.autoreload);
    }
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, step.dac);

    if (!sample_ok) {
        spi_error_count++;
    } else if (stream_on) {
#ifdef ACQ_STREAM_CH1
        if (dual) {
            stream_push(raw, step.phase | STREAM_TAG_DUAL);
            if (sample1_ok) {
                stream_push(raw1, step.phase | STREAM_TAG_DUAL | STREAM_TAG_CH1);
            }
        } else {
            stream_push(raw, step.phase);
        }
#else
        stream_push(raw, step.phase);
#endif
    }
    if (exc_runner_api_sample(&runner, &step, sample_ok, raw, &lockin_result)) {
        lockin_ready = true;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM2) {
        return;
    }
    tim2_tick(htim);
    // L'HAL azzera UIF prima della callback: se e' gia' di nuovo alto, il tick successivo e'
    // scaduto mentre l'ISR girava (tick troppo veloce per la durata dell'ISR). I campioni
    // restano allineati alle tabelle, ma l'eccitazione non e' piu' regolare.
    if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
        exc_runner_api_mark_overrun(&runner);
        overrun_count++;
    }
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
