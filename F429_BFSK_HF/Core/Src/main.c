/* USER CODE BEGIN Header */
/* F429_BFSK_HF */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <stddef.h>
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
DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
#define TABLE_SIZE 		32U
#define BIT_MS 			10U
#define PRE_HIGH_MS		50U
#define PRE_ZERO_MS		50U
#define POST_ZERO_MS	50U
#define GAP_MS			500U

// choose which BFSK pair you want
typedef enum {
	PAIR_30_31 = 0,
	PAIR_45_46 = 1
} FreqPair;

static volatile FreqPair g_pair = PAIR_30_31; // <- change this to change the output frequency pair

#define DAC_Vref_V		3.34f
#define DAC_FULLSCALE	4095.0f
#define DAC_1V68_CODE	((uint16_t)((1.68f / DAC_Vref_V) * DAC_FULLSCALE))

static const uint16_t sineTable[TABLE_SIZE] = {
    2048,2447,2831,3185,3495,3750,3939,4056,
    4095,4056,3939,3750,3495,3185,2831,2447,
    2048,1648,1264, 910, 600, 345, 156,  39,
       0,  39, 156, 345, 600, 910,1264,1648
};

// Tx framing state machine
typedef enum{
	ST_PRE_HIGH = 0,
	ST_PRE_ZERO,
	ST_DATA,
	ST_POST_ZERO,
	ST_GAP
} TxState;

static TxState st = ST_PRE_HIGH;
static uint32_t stTick = 0;
static uint32_t lastBitTick =0;
static uint32_t bitIndex = 0;

// Data buffet: ASCII -> bit string
#define MAX_ASCII_LEN 	64
#define MAX_BITS_LEN	(MAX_ASCII_LEN * 8)

static const char userText[MAX_ASCII_LEN + 1] = "AMAC"; // change the text here
static char bitString[MAX_BITS_LEN + 1];				// converted binary string

static uint32_t tim6_clk_hz = 0;

// build bit string from ascii
static void ascii_to_bits(const char *in, char *out, size_t out_cap)
{
	size_t w = 0;
	for (size_t i = 0; in[i] != '\0'; i++)
	{
		uint8_t c = (uint8_t)in[i];
		for (int b = 7; b >= 0; b--)
		{
			if (w + 1 >= out_cap) {out[w] = '\0'; return; }
			out[w++] = ((c >> b) & 1U) ? '1': '0';
		} /* end inner for */
	} /* end outer for */
	out[w] = '\0';
} /* end function ascii_to_ascii*/

// read TIM6 timer clock (APB1 timer clock)
static uint32_t get_tim6_clock_hz(void)
{
	// Cube/HAL: PCLK1 may be divided; timer clock doubles if APB prescaler != 1
	uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
	uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
	uint32_t apb1_presc = (ppre1 < 4U) ? 1U : (1U << (ppre1 - 3U));
	return (apb1_presc == 1U) ? pclk1 : (2U * pclk1);
} // end function get_tim6_clock_hz

// pick f0/f1 based on pair
static inline uint32_t pair_f0_hz(FreqPair p) { return (p == PAIR_30_31) ? 30000U : 45000U;}
static inline uint32_t pair_f1_hz(FreqPair p) { return (p == PAIR_30_31) ? 31000U : 46000U;}

// Core: compute ARR for desired f_out with TABLE_SIZE samples/cycle
static uint32_t calc_arr_for_fout(uint32_t fout_hz)
{
	// f_update = f_out * TABLE_SIZE
	uint32_t f_update = fout_hz * TABLE_SIZE;

	uint32_t psc = htim6.Init.Prescaler; // using CubeMX PSC
	if (f_update == 0U) f_update = 1U;

	// ARR = tim_clk / ((PSC+1)*f_update) -1
	uint32_t arr = (tim6_clk_hz / ((psc + 1U) * f_update));
	if (arr > 0U) arr -= 1U;

	// guardrails
	if (arr < 1U) arr = 1U;
	if (arr > 0xFFFFU) arr = 0xFFFFU;

	return arr;
} // end function calc_arr_for_fout

// Core: apply ARR safely (dynamic)
static void tim6_set_arr(uint32_t arr)
{
	__HAL_TIM_DISABLE(&htim6);
	__HAL_TIM_SET_AUTORELOAD(&htim6 , arr);
	__HAL_TIM_SET_COUNTER(&htim6 , 0);

	// Force register reload immediately
	HAL_TIM_GenerateEvent(&htim6 , TIM_EVENTSOURCE_UPDATE);

	__HAL_TIM_ENABLE(&htim6);
}

// Core: set frequency from one bit ('0'/'1')
static void set_symbol_freq(char b)
{
	uint32_t f = (b == '0') ? pair_f0_hz(g_pair) : pair_f1_hz(g_pair);
	uint32_t arr = calc_arr_for_fout(f);
	tim6_set_arr(arr);
} // end function set_symbol_freq

static TxState st_prev = 0xFF;
static inline uint8_t is_enter_state(TxState s)
{
	if (st != st_prev) {st_prev = st; return 1;}
	return 0;
} // end function is_enter_state
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  // 1) build bit string once
  ascii_to_bits(userText , bitString , sizeof(bitString));

  // 2) start DAC with DMA (circular waveform table)
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1 , (uint32_t*)sineTable , TABLE_SIZE , DAC_ALIGN_12B_R);

  // 3) start TIM6 (TIM6 TRGO should trigger DAC)
  HAL_TIM_Base_Start(&htim6);

  // 4) cache timer clock for ARR calculation
  tim6_clk_hz = get_tim6_clock_hz();

  // 5) enter PRE_HIGH: disable timer updates, force DC 1.68V
  __HAL_TIM_DISABLE(&htim6);
  HAL_DAC_SetValue(&hdac , DAC_CHANNEL_1 , DAC_ALIGN_12B_R , DAC_1V68_CODE);

  st = ST_PRE_HIGH;
  stTick = HAL_GetTick();
  lastBitTick = stTick;
  bitIndex = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t now = HAL_GetTick();
	  switch (st)
	  {
	  case ST_PRE_HIGH:
		  if (is_enter_state(ST_PRE_HIGH))
		  {
			  __HAL_TIM_DISABLE(&htim6);				// ensure no trigger
			  HAL_DAC_Stop_DMA(&hdac , DAC_CHANNEL_1);	// (recommended) avoid DMA residue
			  HAL_DAC_SetValue(&hdac , DAC_CHANNEL_1 , DAC_ALIGN_12B_R , DAC_1V68_CODE);
		  } // end if

		  if (now - stTick >= PRE_HIGH_MS)
		  {
			  st = ST_PRE_ZERO;
			  stTick = now;
		  } // end if

		  break;

	  case ST_PRE_ZERO:
		  if (is_enter_state(ST_PRE_ZERO))
		  {
			  __HAL_TIM_DISABLE(&htim6);
			  HAL_DAC_Stop_DMA(&hdac , DAC_CHANNEL_1);
			  HAL_DAC_SetValue(&hdac , DAC_CHANNEL_1 , DAC_ALIGN_12B_R , 0);
		  } // end if

		  if (now - stTick >= PRE_ZERO_MS)
		  {
			  st = ST_DATA;
			  stTick = now;
		  } // end if

		  break;

	  case ST_DATA:
		  if (is_enter_state(ST_DATA))
		  {
			  bitIndex = 0;
			  lastBitTick = now;

			  // 1) reset LUT/DMA pointer so DATA always from table[0]
			  HAL_DAC_Stop_DMA(&hdac , DAC_CHANNEL_1);
			  HAL_DAC_Start_DMA(&hdac , DAC_CHANNEL_1 , (uint32_t*)sineTable , TABLE_SIZE , DAC_ALIGN_12B_R);

			  // 2) reset timer counter and enable
			  __HAL_TIM_SET_COUNTER(&htim6 , 0);
			  __HAL_TIM_ENABLE(&htim6);

			  // 3) set first symbol
			  if (bitString[0] == '\0')
			  {
				  __HAL_TIM_DISABLE(&htim6);
				  HAL_DAC_Stop_DMA(&hdac , DAC_CHANNEL_1);
				  HAL_DAC_SetValue(&hdac , DAC_CHANNEL_1 , DAC_ALIGN_12B_R , 0);

				  st = ST_POST_ZERO;
				  stTick = now;
			  } // end if
			  else
			  {
				  set_symbol_freq(bitString[0]);
			  } // end else
		  } // end if

		  // change freq every BIT_MS

		  if (now - lastBitTick >= BIT_MS)
		  {
			  lastBitTick += BIT_MS;
			  bitIndex++;

			  char b = bitString[bitIndex];
			  if (b == '\0')
			  {
				  __HAL_TIM_DISABLE(&htim6);
				  HAL_DAC_Stop_DMA(&hdac , DAC_CHANNEL_1);
				  HAL_DAC_SetValue(&hdac , DAC_CHANNEL_1 , DAC_ALIGN_12B_R , 0);

				  st = ST_POST_ZERO;
				  stTick = now;
			  } // end if
			  else
			  {
				  set_symbol_freq(b);
			  } // end else
		  } // end if

		  break;

	  case ST_POST_ZERO:
		  if (is_enter_state(ST_POST_ZERO))
		  {
			  __HAL_TIM_DISABLE(&htim6);
			  HAL_DAC_Stop_DMA(&hdac , DAC_CHANNEL_1);
			  HAL_DAC_SetValue(&hdac , DAC_CHANNEL_1 , DAC_ALIGN_12B_R , 0);
		  } // end if

		  if (now - stTick >= POST_ZERO_MS)
		  {
			  st =  ST_GAP;
			  stTick = now;
		  } // end if

		  break;

	  case ST_GAP:
		  if (is_enter_state(ST_GAP))
		  {
			  __HAL_TIM_DISABLE(&htim6);
			  HAL_DAC_Stop_DMA(&hdac , DAC_CHANNEL_1);
			  HAL_DAC_SetValue(&hdac , DAC_CHANNEL_1 , DAC_ALIGN_12B_R , 0);
		  } // end if

		  if (now - stTick >= GAP_MS)
		  {
			  st = ST_PRE_HIGH;
			  stTick = now;
		  } // end if
		  break;

	  default:
		  __HAL_TIM_DISABLE(&htim6);
		  HAL_DAC_Stop_DMA(&hdac , DAC_CHANNEL_1);
		  HAL_DAC_SetValue(&hdac , DAC_CHANNEL_1 , DAC_ALIGN_12B_R , 0);
		  break;

	  } // end switch
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
