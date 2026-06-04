/* USER CODE BEGIN Header */
/* F7676_midc_test */
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "fatfs.h"
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

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
#define RAW_INTERVAL_SEC      600   // 10 minutes
#define AVG_INTERVAL_SEC      3600  // 1 hour

#define DS3231_ADDR           (0x68 << 1)

#define MPU6050_ADDR          (0x68 << 1)
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_XOUT_H  0x3B

#define SBE_RX_BUFFER_SIZE    256

typedef struct
{
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
  uint8_t day;
  uint8_t date;   // which day in a week
  uint8_t month;
  uint8_t year;
} DS3231_Time_t;

typedef struct
{
  float ax_g;
  float ay_g;
  float az_g;

  float gx_dps;
  float gy_dps;
  float gz_dps;

  float temp_c;
} MPU6050_Data_t;

typedef struct
{
  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;

  float temp_bmp;
  float temp_sbe;
  float pressure_sbe;
} SensorData_t;

typedef struct
{
  uint32_t count;

  double ax_sum;
  double ay_sum;
  double az_sum;

  double gx_sum;
  double gy_sum;
  double gz_sum;

  double temp_bmp_sum;
  double temp_sbe_sum;
  double pressure_sbe_sum;
} AvgBuffer_t;

FATFS fs;
FIL file;
FRESULT fres;

DS3231_Time_t rtc_time;
MPU6050_Data_t mpu;
SensorData_t sensor_data;
AvgBuffer_t avg_buf;

uint32_t last_raw_sec = 0;
uint32_t last_avg_sec = 0;

uint32_t raw_index = 0;
uint32_t avg_index = 0;

char uart_msg[512];
char raw_line[256];
char avg_line[256];
char sbe_raw[SBE_RX_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */
void UART3_Print(const char *msg);

uint8_t BCD_To_Dec(uint8_t bcd);
HAL_StatusTypeDef DS3231_ReadTime(DS3231_Time_t *time);
uint32_t DS3231_ToSecond(DS3231_Time_t *t);

HAL_StatusTypeDef MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_Read(MPU6050_Data_t *data);

void SBE39_WakeUP(void);
HAL_StatusTypeDef SBE39_SendCommand(const char *cmd , char *buffer , uint16_t buffer_size , uint32_t timeout_ms);
uint8_t SBE39_ParseTS(char *raw , float *temp , float *pressure);

HAL_StatusTypeDef Read_All_Sensors(SensorData_t *d);

void Avg_Reset(void);
void Avg_AddSample(SensorData_t *d);

void SD_AppendLine_WithHeader(const char *filename , const char *header , const char *line);
void SD_WriteRaw(SensorData_t *d, DS3231_Time_t *t);
void SD_WriteAvg_And_SendUART(DS3231_Time_t *t);
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
  MX_ETH_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  UART3_Print("System start\r\n");

  if (MPU6050_Init() == HAL_OK)
  {
    UART3_Print("MPU6050 init OK\r\n");
  } // end if mpu init ok
  else
  {
    UART3_Print("MPU6050 init ERROR\r\n");
  } // end else

  if (DS3231_ReadTime(&rtc_time) == HAL_OK)
  {
    uint32_t now_sec = DS3231_ToSecond(&rtc_time);

    last_raw_sec = now_sec - (now_sec % RAW_INTERVAL_SEC);    // Align raw logging to the nearest previous sampling boundary
    last_avg_sec = now_sec - (now_sec % AVG_INTERVAL_SEC);    // Align avgerging to the nearest averaging boundary

    UART3_Print("DS3231 init OK\r\n");
  } // end if DS3231 readtime ok 
  else
  {
    UART3_Print("DS3231 init ERROR\r\n");
  } // end else

  Avg_Reset();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (DS3231_ReadTime(&rtc_time) == HAL_OK)
    {
      uint32_t now_sec = DS3231_ToSecond(&rtc_time);

      if (now_sec - last_raw_sec >= RAW_INTERVAL_SEC)
      {
        last_raw_sec += RAW_INTERVAL_SEC;

        if (Read_All_Sensors(&sensor_data) == HAL_OK)
        {
          SD_WriteRaw(&sensor_data , &rtc_time);
          Avg_AddSample(&sensor_data);

          UART3_Print("RAW WRITE OK\r\n");
        } // end if read all sensor ok
        else
        {
          UART3_Print("SENSOR READ ERROR\r\n");
        } // end else
      } // end if reach raw interval sec

      if (now_sec - last_avg_sec >= AVG_INTERVAL_SEC)
      {
        last_avg_sec += AVG_INTERVAL_SEC;

        SD_WriteAvg_And_SendUART(&rtc_time);
        Avg_Reset();
      } // end if reach abg interval sec
    } // end DS3231_ReadTime ok
    
    HAL_Delay(100);
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
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
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

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
void UART3_Print(const char *msg)
{
  HAL_UART_Transmit(&huart3 , (uint8_t *)msg , strlen(msg) , 100);
} // end function UART3_Print

uint8_t BCD_To_Dec(uint8_t bcd)
{
  return ((bcd >> 4) * 10) + (bcd & 0x0F);
} // end function BCD_To_Dec

HAL_StatusTypeDef DS3231_ReadTime(DS3231_Time_t *time)
{
  uint8_t raw[7];

  if (HAL_I2C_Mem_Read(&hi2c2 , DS3231_ADDR , 0x00 , I2C_MEMADD_SIZE_8BIT , raw , 7 , 100) != HAL_OK)
  {
    return HAL_ERROR;
  } // end if hal_I2C_Mem_Read not ok

  time->sec     = BCD_To_Dec(raw[0] & 0x7F);
  time->min     = BCD_To_Dec(raw[1] & 0x7F);
  time->hour    = BCD_To_Dec(raw[2] & 0x3F);
  time->day     = BCD_To_Dec(raw[3] & 0x07);
  time->date    = BCD_To_Dec(raw[4] & 0x3F);
  time->month   = BCD_To_Dec(raw[5] & 0x1F);
  time->year    = BCD_To_Dec(raw[6]);

  return HAL_OK;
} // end function DS3231_ReadTime

uint32_t DS3231_ToSecond(DS3231_Time_t *t)
{
  return (uint32_t)t->hour * 3600UL + (uint32_t)t->min * 60UL + (uint32_t)t->sec;
} // end function DS3231_ToSecond

HAL_StatusTypeDef MPU6050_Init(void)
{
  uint8_t data = 0x00;

  if (HAL_I2C_Mem_Write(&hi2c1 , MPU6050_ADDR , MPU6050_PWR_MGMT_1 , 1 , &data , 1 , 100) != HAL_OK)
  {
    return HAL_ERROR;
  } // end if HAL_I2C_Mem_Write not ok

  HAL_Delay(100);

  return HAL_OK;
} // end function MPU6050_Init

HAL_StatusTypeDef MPU6050_Read(MPU6050_Data_t *data)
{
  uint8_t raw[14];

  if (HAL_I2C_Mem_Read(&hi2c1 , MPU6050_ADDR , MPU6050_ACCEL_XOUT_H , 1 , raw , 14 , 100) != HAL_OK)
  {
    return HAL_ERROR;
  } // end if HAL_I2CMem_Read not ok

  int16_t ax_raw = (int16_t)(raw[0] << 8 | raw[1]);
  int16_t ay_raw = (int16_t)(raw[2] << 8 | raw[3]);
  int16_t az_raw = (int16_t)(raw[4] << 8 | raw[5]);
  
  int16_t temp_raw = (int16_t)(raw[6] << 8 | raw[7]);

  int16_t gx_raw = (int16_t)(raw[8] << 8 | raw[9]);
  int16_t gy_raw = (int16_t)(raw[10] << 8 | raw[11]);
  int16_t gz_raw = (int16_t)(raw[12] << 8 | raw[13]);

  data->ax_g = ax_raw / 16384.0f;
  data->ay_g = ay_raw / 16384.0f;
  data->az_g = az_raw / 16384.0f;

  data->gx_dps = gx_raw / 131.0f;
  data->gy_dps = gy_raw / 131.0f;
  data->gz_dps = gz_raw / 131.0f;

  data->temp_c = (temp_raw / 340.0f) + 36.53f;

  return HAL_OK;
} // end function MPU6050_Read

void SBE39_WakeUP(void)
{
  char wake[] = "\r";

  HAL_UART_Transmit(&huart2 , (uint8_t *)wake , strlen(wake) , 100);
  HAL_Delay(4000);
} // end function SBE30_WakeUp

HAL_StatusTypeDef SBE39_SendCommand(const char *cmd , char *buffer , uint16_t buffer_size , uint32_t timeout_ms)
{
  uint8_t ch;
  uint16_t idx = 0;
  uint32_t start_time;

  memset(buffer , 0 , buffer_size);

  SBE39_WakeUP();

  HAL_UART_Transmit(&huart2 , (uint8_t *)cmd , strlen(cmd) , 100);

  start_time = HAL_GetTick();

  while ((HAL_GetTick() - start_time < timeout_ms))
  {
    if (HAL_UART_Receive(&huart2 , &ch , 1 , 20) == HAL_OK)
    {
      if (idx < buffer_size - 1)
      {
        buffer[idx++] = ch;
      } // end if idx smaller than buffer_size
      start_time = HAL_GetTick();
    } // end if HAL_UART_Receive ok
  } // end while
  
  buffer[idx] = '\0';

  if (idx > 0)
  {
    return HAL_OK;
  } // end if idx > 0

  strcpy(buffer , "NO_DATA");
  return HAL_TIMEOUT;
} // end function SBE39_SendCommand

uint8_t SBE39_ParseTS(char *raw, float *temp, float *pressure)
{
  char *p = raw;
  char *endptr;

  // Find first numeric character
  while (*p)
  {
    if ((*p >= '0' && *p <= '9') ||
        (*p == '-') ||
        (*p == '+'))
    {
        break;
    }

    p++;
  }

  if (*p == '\0')
  {
      return 0;
  }

  // Parse temperature
  *temp = strtof(p, &endptr);

  if (p == endptr)
  {
      return 0;
  }

  p = endptr;

  // Find next numeric character
  while (*p)
  {
      if ((*p >= '0' && *p <= '9') ||
          (*p == '-') ||
          (*p == '+'))
      {
          break;
      }

      p++;
  }

  if (*p == '\0')
  {
      return 0;
  }

  // Parse pressure
  *pressure = strtof(p, &endptr);

  if (p == endptr)
  {
      return 0;
  }

  return 1;
} // end function SBE39_ParseTS

HAL_StatusTypeDef Read_All_Sensors(SensorData_t *d)
{
  if (MPU6050_Read(&mpu) != HAL_OK)
  {
    return HAL_ERROR;
  } // end if MPU6050_Read not OK

  d->ax = mpu.ax_g;
  d->ay = mpu.ay_g;
  d->az = mpu.az_g;
  d->gx = mpu.gx_dps;
  d->gy = mpu.gy_dps;
  d->gz = mpu.gz_dps;
  d->temp_bmp = mpu.temp_c;
  
  if (SBE39_SendCommand("TS\r" , sbe_raw , sizeof(sbe_raw) , 5000) == HAL_OK)
  {
    snprintf(uart_msg , sizeof(uart_msg) , "SBE RAW: [%s]\r\n" , sbe_raw);
    UART3_Print(uart_msg);

    if (!SBE39_ParseTS(sbe_raw , &d->temp_sbe , &d->pressure_sbe))
    {
      UART3_Print("SBE PARSE ERROR\r\n");

      snprintf(uart_msg , sizeof(uart_msg) , "PARSE TEST sscanf=%d\r\n", sscanf(sbe_raw, "%f,%f", &sensor_data.temp_sbe, &sensor_data.pressure_sbe));

      UART3_Print(uart_msg);
      d->temp_sbe = -999.0f;
      d->pressure_sbe = -999.0f;
    } // end if not SBE39_ParseTS
    else
    {
      snprintf(uart_msg , sizeof(uart_msg) , "SBE PARSE OK: temp=%.4f, pressure=%.4f\r\n",  d->temp_sbe , d->pressure_sbe);
      UART3_Print(uart_msg);
    } // end else
  } // end if SBE39_SendCommand is OK
  else
  {
    d->temp_sbe = -999.0f;
    d->pressure_sbe = -999.0f;
  } // end else

  return HAL_OK;
} // end function Read_All_Sensors

void Avg_Reset(void)
{
  memset(&avg_buf , 0 , sizeof(avg_buf));
} // end function Avg_Reset

void Avg_AddSample(SensorData_t *d)
{
  avg_buf.count++;
  avg_buf.ax_sum += d->ax;
  avg_buf.ay_sum += d->ay;
  avg_buf.az_sum += d->az;
  
  avg_buf.gx_sum += d->gx;
  avg_buf.gy_sum += d->gy;
  avg_buf.gz_sum += d->gz;

  avg_buf.temp_bmp_sum += d->temp_bmp;
  avg_buf.temp_sbe_sum += d->temp_sbe;
  avg_buf.pressure_sbe_sum += d->pressure_sbe;
} // end function Avg_AddSample

void SD_AppendLine_WithHeader(const char *filename , const char *header , const char *line)
{
  UINT bw;
  FILINFO fno;
  uint8_t need_header = 0;

  fres = f_mount(&fs , "" , 1);

  if (fres != FR_OK)
  {
    snprintf(uart_msg , sizeof(uart_msg) , "SD mount ERROR: %d\r\n" , fres);
    UART3_Print(uart_msg);
    return;
  } // end if fres not ok

  fres = f_stat(filename , &fno);
  
  if (fres == FR_NO_FILE)
  {
    need_header = 1;
  } // end if fres no file
  else if (fres == FR_OK && fno.fsize == 0)
  {
    need_header = 1;
  } // end else if

  fres = f_open(&file , filename , FA_OPEN_ALWAYS | FA_WRITE);

  if (fres != FR_OK)
  {
    snprintf(uart_msg , sizeof(uart_msg) , "File open ERROR %s: %d\r\n" , filename , fres);
    UART3_Print(uart_msg);
    f_mount(NULL , "" , 1);
    return;
  } // end if fres not ok

  f_lseek(&file , f_size(&file));

  if (need_header)
  {
    f_write(&file , header , strlen(header) , &bw);
  } // end if need header

  f_write(&file , line , strlen(line) , &bw);
  f_sync(&file);
  f_close(&file);
  f_mount(NULL , "" , 1);
} // end function SD_AppendLine_WithHeader

void SD_WriteRaw(SensorData_t *d , DS3231_Time_t *t)
{
  raw_index++;

  snprintf(raw_line , sizeof(raw_line) , "20%02d-%02d-%02d %02d:%02d:%02d,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.4f,%.4f\r\n",
          t->year,
          t->month,
          t->day,
          t->hour,
          t->min,
          t->sec,
          raw_index,
          d->ax,
          d->ay,
          d->az,
          d->gx,
          d->gy,
          d->gz,
          d->temp_bmp,
          d->temp_sbe,
          d->pressure_sbe);

  SD_AppendLine_WithHeader("raw_log.csv" , "rtc_time,index,ax,ay,az,gx,gy,gz,temp_bmp,temp_sbe,pressure_sbe\r\n" , raw_line);
} // end function SD_WriteRaw

void SD_WriteAvg_And_SendUART(DS3231_Time_t *t)
{
  if (avg_buf.count == 0)
  {
    return;
  } // end if avg count is 0

  float ax_avg = avg_buf.ax_sum / avg_buf.count;
  float ay_avg = avg_buf.ay_sum / avg_buf.count;
  float az_avg = avg_buf.az_sum / avg_buf.count;

  float gx_avg = avg_buf.gx_sum / avg_buf.count;
  float gy_avg = avg_buf.gy_sum / avg_buf.count;
  float gz_avg = avg_buf.gz_sum / avg_buf.count;

  float temp_bmp_avg = avg_buf.temp_bmp_sum / avg_buf.count;
  float temp_sbe_avg = avg_buf.temp_sbe_sum / avg_buf.count;
  float pressure_sbe_avg = avg_buf.pressure_sbe_sum / avg_buf.count;

  avg_index++;

  snprintf(avg_line , sizeof(avg_line) , "20%02d-%02d-%02d %02d:%02d:%02d,%lu,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.4f,%.4f\r\n",
          t->year,
          t->month,
          t->day,
          t->hour,
          t->min,
          t->sec,
          avg_index,
          avg_buf.count,
          ax_avg,
          ay_avg,
          az_avg,
          gx_avg,
          gy_avg,
          gz_avg,
          temp_bmp_avg,
          temp_sbe_avg,
          pressure_sbe_avg);

  SD_AppendLine_WithHeader("avg_log.csv" , "rtc_time,avg_index,count,ax_avg,ay_avg,az_avg,gx_avg,gy_avg,gz_avg,temp_bmp_avg,temp_sbe_avg,pressure_sbe_avg\r\n",
    avg_line);

  snprintf(uart_msg , sizeof(uart_msg) , "AVG,%s", avg_line);
  HAL_UART_Transmit(&huart3 , (uint8_t *)uart_msg , strlen(uart_msg) , 100);
} // end function SD_WriteAvg_And_SendUART
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
