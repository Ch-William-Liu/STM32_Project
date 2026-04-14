/* USER CODE BEGIN Header */
/* F767_SignalGen */
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DAC_FS          196000.0f
#define BUF_LEN         4096
#define HALF_BUF_LEN    (BUF_LEN / 2)
#define URART_RX_LEN    64
#define PI_F            3.14159265358979f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint16_t dacBuf[BUF_LEN];

uint8_t uartRxByte;
char uartCmd[URART_RX_LEN];
volatile uint16_t uartIdx = 0;
volatile bool uartCmdReady = false;

volatile bool chirpRunning = false;
volatile bool chirpStopRequested = false;

volatile uint32_t totalSamples = 0;
volatile uint32_t currentSampleIndex = 0;

volatile float chirpFlow = 36000.0f;        // default chirp f_low
volatile float chirpFhigh = 42000.0f;       // default chirp f_high
volatile float chirpDurationMs = 300.0f;    // default chirp duration (ms)
volatile float chirpDurationSec = 0.3f;     // default chirp duration (sec)
volatile float chirpSlope = 0.0f;

volatile uint16_t dacMid = 2048;
volatile float chirpAmp = 1800.0f;          // keep below full scale

volatile bool chirpFinishedFlag = false;
volatile uint32_t halfCallbackCount = 0;
volatile uint32_t fullCallbackCount = 0;

volatile uint32_t chirpStartTick = 0;
volatile uint32_t chirpEndTick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_ETH_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */
// function prototype
void UART_BeginReceive_IT(void);
void ProcessUartCommand(void);
void ParseAndStartChirp(char *cmd);

void StartChirp(float flow , float fhigh , float durationMs);
void StopChirp(void);

void FillBufferRange(uint16_t *buf , uint32_t startIdx , uint32_t len);
uint16_t GenerateOneSample(uint32_t n);

int __io_putchar(int ch);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart3 , (uint8_t *)&ch , 1 , HAL_MAX_DELAY);
  return ch;
} // end function __io_putchar

void UART_BeginReceive_IT(void)
{
  HAL_UART_Receive_IT(&huart3 , &uartRxByte , 1);
} // end function UART_BeginReceive_IT

void ProcessUartCommand(void)
{
  if (uartCmdReady)
  {
    uartCmdReady = false;
    ParseAndStartChirp(uartCmd);
    memset(uartCmd , 0 , sizeof(uartCmd));
    uartIdx = 0;
  } // end if (uartCmdReady)
} // end function ProcessUartCommand

void ParseAndStartChirp(char *cmd)
{
  uint32_t f1 , f2 , durMs;

  if (sscanf(cmd , "chirp %lu %lu %lu" , &f1 , &f2 , &durMs) == 3)
  {
    if (f1 < 500.0f || f1 > 50000.0f || f2 < 500.0f || f2 > 50000.0f || durMs < 300.0f || durMs > 1000.0f)
    {
      printf("Invalid range.\r\n");
      printf("Use: chirp <f_low> <f_high> <duration_ms>\r\n");
      printf("Example: chirp 36000 42000 300\r\n");

      return;
    } // end if (checking the parameter is in range or not)

    StartChirp(f1 , f2 , durMs);        // Start generate chirp and output
  } // end if (sscanf == 3)
  else
  {
    printf("Unknown command.\r\n");
    printf("Use: chirp <f_low> <f_high> <duration_ms>\r\n");
  } // end else (unknown command)
} // end function ParseAndStartChirp

void StartChirp(float flow , float fhigh , float durationMs)
{
  HAL_StatusTypeDef st_tim, st_dac;


  StopChirp();

  chirpFlow = flow;
  chirpFhigh = fhigh;
  chirpDurationMs = durationMs;
  chirpDurationSec = durationMs / 1000.0f;
  chirpSlope = (chirpFhigh - chirpFlow) / chirpDurationSec;

  totalSamples = (uint32_t)(DAC_FS * chirpDurationSec);
  currentSampleIndex = 0;
  chirpStopRequested = false;
  chirpRunning = true;
  halfCallbackCount = 0;
  fullCallbackCount = 0;
  chirpEndTick = 0;
  chirpFinishedFlag = false;

  FillBufferRange(dacBuf , 0 , HALF_BUF_LEN);
  FillBufferRange(&dacBuf[HALF_BUF_LEN] , HALF_BUF_LEN , HALF_BUF_LEN);

  st_tim = HAL_TIM_Base_Start(&htim6);
  st_dac = HAL_DAC_Start_DMA(&hdac , DAC_CHANNEL_1 , (uint32_t *)dacBuf , BUF_LEN , DAC_ALIGN_12B_R);

  chirpStartTick = HAL_GetTick();

  printf("Start chirp: %lu -> %lu Hz, %lu ms, samples = %lu\r\n",
         (uint32_t)flow,
         (uint32_t)fhigh,
         (uint32_t)durationMs,
         totalSamples);

  printf("TIM start = %d, DAC DMA start = %d\r\n", st_tim, st_dac);
  HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
} // end function StartChirp

void StopChirp(void)
{
  HAL_DAC_Stop_DMA(&hdac , DAC_CHANNEL_1);
  HAL_TIM_Base_Stop(&htim6);
  HAL_DAC_SetValue(&hdac , DAC_CHANNEL_1 , DAC_ALIGN_12B_R , dacMid);

  chirpRunning = false;
  chirpStopRequested = false;
  currentSampleIndex = 0;
  HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
} // end function StopChirp

void FillBufferRange(uint16_t *buf , uint32_t startidx , uint32_t len)
{
  for (uint32_t i = 0; i < len; i++)
  {
    uint32_t n = currentSampleIndex;

    if (n < totalSamples && !chirpStopRequested)
    {
      buf[i] = GenerateOneSample(n);
      currentSampleIndex++;
    } // end if
    else
    {
      buf[i] = dacMid;
      chirpStopRequested = true; 
    } // end else
  } // end for
} // end function FillBufferRange

uint16_t GenerateOneSample(uint32_t n)
{
  float Nminus1;
  float t;
  float phase;
  float w;
  float x;
  float y;
  int32_t dacValue;

  if (totalSamples < 2)
  {
    return dacMid;
  } // end if

  Nminus1 = (float)(totalSamples - 1);
  t = (float)n / DAC_FS;

  // hanning window
  w = 0.5f * (1.0f - cosf((2.0f * PI_F * (float)n) / Nminus1));

  // Linear Chirp
  phase = 2.0 * PI_F * (chirpFlow * t + 0.5f * chirpSlope * t * t);

  x = sinf(phase);
  y = chirpAmp * w * x;

  dacValue = (int32_t)(dacMid + y);

  if (dacValue < 0) dacValue = 0;
  if (dacValue > 4095) dacValue = 4095;

  return (uint16_t)dacValue;
} // end function GenerateOneSample
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_ETH_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac , DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac , DAC_CHANNEL_1 , DAC_ALIGN_12B_R , dacMid);

  UART_BeginReceive_IT();


  printf("System ready.\r\n");
  printf("Use: chirp <f_low> <f_high> <duration_ms>\r\n");
  printf("Example: chirp 36000 42000 300\r\n");

  HAL_DAC_SetValue(&hdac , DAC_CHANNEL_1 , DAC_ALIGN_12B_R , dacMid);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t lastPrintTick = 0;

    ProcessUartCommand();
    if (chirpFinishedFlag)
    {
      chirpFinishedFlag = false;
      printf("Chirp finished. elasped = %lu ms.\r\n" , chirpEndTick - chirpStartTick);
    }

    if (chirpRunning && (HAL_GetTick() - lastPrintTick >= 200))
    {
      lastPrintTick = HAL_GetTick();

      printf("run=%d, stopReq=%d, idx=%lu, total=%lu, half=%lu, full=%lu\r\n",
            chirpRunning,
            chirpStopRequested,
            currentSampleIndex,
            totalSamples,
            halfCallbackCount,
            fullCallbackCount);
    }
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  htim6.Init.Period = 488;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    HAL_UART_Transmit(&huart3, &uartRxByte, 1, HAL_MAX_DELAY);   // echo back
    if (uartRxByte == '\r' || uartRxByte == '\n')
    {
      if (uartIdx > 0)
      {
        uartCmd[uartIdx] = '\0';
        uartCmdReady = true;
      } // end if
      uartIdx = 0;
    } // end if
    else
    {
      if (uartIdx < (URART_RX_LEN - 1))
      {
        uartCmd[uartIdx++] = (char)uartRxByte;
      }
    } // end else

    HAL_UART_Receive_IT(&huart3 , &uartRxByte , 1);
  } // end if
} // end function HAL_UART_RxCpltCallback

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  halfCallbackCount++;

  if (chirpRunning)
  {
    FillBufferRange(dacBuf, 0, HALF_BUF_LEN);
  } // end if 
} // end function HAL_DAC_ConvHalfCpltCallbackCh1

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  fullCallbackCount++;  

  if (chirpRunning)
  {
    FillBufferRange(&dacBuf[HALF_BUF_LEN], HALF_BUF_LEN, HALF_BUF_LEN);

    if (chirpStopRequested)
    {
      chirpEndTick = HAL_GetTick();
      StopChirp();
      chirpFinishedFlag = true;
    }
  }
} // end function HAL_DAC_ConvCpltCallbackCh1
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
