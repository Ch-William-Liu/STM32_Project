/* USER CODE BEGIN Header */
/* F767_MIRDC_TxNode_MCUVersion */
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c4;

SD_HandleTypeDef hsd1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define MPU6050_ADDR  (0x69 << 1)

static uint8_t mpu_ready = 0;

#define BQ76942_ADDR  (0x08 << 1)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C4_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef MPU6050_Init(void)
{
  uint8_t value;

  value = 0x01;

  if (HAL_I2C_Mem_Write(&hi2c4, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &value, 1, 100) != HAL_OK)
  {
    return HAL_ERROR;
  } // end if mem write 0x01 not OK

  HAL_Delay(100);

  // Digital Low-pass filter
  value = 0x03;
  if (HAL_I2C_Mem_Write(&hi2c4, MPU6050_ADDR, 0x1A, I2C_MEMADD_SIZE_8BIT, &value, 1, 100) != HAL_OK)
  {
    return HAL_ERROR;
  } // end if mem write 0x03 not OK

  HAL_Delay(100);

  // Gyro
  value = 0x00;
  if (HAL_I2C_Mem_Write(&hi2c4, MPU6050_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, &value, 1, 100) != HAL_OK)
  {
    return HAL_ERROR;
  } // return if mem write gyro value not ok

  HAL_Delay(100);

  // ACC
  value = 0x00;
  if (HAL_I2C_Mem_Write(&hi2c4, MPU6050_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &value, 1, 100) != HAL_OK)
  {
    return HAL_ERROR;
  } // return if mem write gyro value not ok

  return HAL_OK;
} // end function MPU6050_Init

HAL_StatusTypeDef MPU6050_Read(float accel[3], float gyro[3], float *temperature)
{
  uint8_t data[14];

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c4, MPU6050_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, data, 14, 100);

  if (status != HAL_OK)
  {
    return status;
  } // end if status not ok

  int16_t ax = (int16_t)((data[0] << 8) | data[1]);
  int16_t ay = (int16_t)((data[2] << 8) | data[3]);
  int16_t az = (int16_t)((data[4] << 8) | data[5]);

  int16_t temp = (int16_t)((data[6] << 8) | data[7]);

  int16_t gx = (int16_t)((data[8] << 8) | data[9]);
  int16_t gy = (int16_t)((data[10] << 8) | data[11]);
  int16_t gz = (int16_t)((data[12] << 8) | data[13]);

  accel[0] = ax / 16384.0f;
  accel[1] = ay / 16384.0f;
  accel[2] = az / 16384.0f;

  gyro[0] = gx / 131.0f;
  gyro[1] = gy / 131.0f;
  gyro[2] = gz / 131.0f;

  *temperature = temp / 340.f + 36.53f;

  return HAL_OK;
} // end function MPU6050_Read
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

  /* New Version PCB SD card power control */
  HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_I2C2_Init();
  MX_I2C4_Init();
  MX_SDMMC1_SD_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  
  // USART3 Start  up test
  const char uart_msg[] = "STM32F767 PCB Boot OK\r\n";

  HAL_UART_Transmit(&huart3, (uint8_t *)uart_msg, sizeof(uart_msg) - 1, HAL_MAX_DELAY);

  // MPU6050 start up
  mpu_ready = (MPU6050_Init() == HAL_OK);
  const char *mpu_msg = mpu_ready ? "[MPU] Initialization OK\r\n" : "[MPU] Initialization FAILED\r\n";

  HAL_UART_Transmit(&huart3, (uint8_t *)mpu_msg, strlen(mpu_msg), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(LED_PG0_GPIO_Port, LED_PG0_Pin);

    const char test_msg[] = "System Running\r\n";

    HAL_UART_Transmit(&huart3, (uint8_t *)test_msg, sizeof(test_msg) - 1, 100);

    HAL_Delay(100);

    // DS3231 connection test
    HAL_StatusTypeDef rtc_status;

    rtc_status = HAL_I2C_IsDeviceReady(&hi2c2, (0x68 << 1), 3, 100);

    char msg[128];

    snprintf(msg, sizeof(msg), "[RTC] status=%d, I2C_error=0x%08lX, I2C_state=%d\r\n", (int)rtc_status, (unsigned long)HAL_I2C_GetError(&hi2c2), (int)HAL_I2C_GetState(&hi2c2));

    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);

    HAL_Delay(100);

    // MPU connection test
    HAL_StatusTypeDef mpu_status;

    mpu_status = HAL_I2C_IsDeviceReady(&hi2c4, (0x69 << 1), 3, 100);
  
    char mpu_msg[128];
  
    snprintf(mpu_msg, sizeof(mpu_msg), "[MPU] status=%d, I2C_error=0x%08lX, I2C_state=%d\r\n", (int)mpu_status, (unsigned long)HAL_I2C_GetError(&hi2c4), (int)HAL_I2C_GetState(&hi2c4));
  
    HAL_UART_Transmit(&huart3, (uint8_t *)mpu_msg, strlen(mpu_msg), 100);

    HAL_Delay(100);
    
    uint8_t who_am_i = 0;

    HAL_StatusTypeDef who_status = HAL_I2C_Mem_Read(&hi2c4, (0x69 << 1), 0x75, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100);

    char who_msg[100];

    snprintf(who_msg, sizeof(who_msg), "[MPU] status=%d, WHO_AM_I=0x%02X, error=0x%08lX\r\n", (int)who_status, who_am_i, (unsigned long)HAL_I2C_GetError(&hi2c4));

    HAL_UART_Transmit(&huart3, (uint8_t *)who_msg, strlen(who_msg), 100);

    HAL_Delay(100);

    // print out 
    if (mpu_ready)
    {
      float accel[3];
      float gyro[3];
      float temperature;
      
      if (MPU6050_Read(accel, gyro, &temperature) == HAL_OK)
      {
        char val_msg[256];

        snprintf(val_msg, sizeof(val_msg), "[MPU] ACC: X=%.2f,Y=%.2f,Z=%.2f g | GYRO: X=%.2f,Y=%.2f,Z=%.2f g | TEMP=%.2f C\r\n", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], temperature);

        HAL_UART_Transmit(&huart3, (uint8_t *)val_msg, strlen(val_msg), 100);
      } // end read MPU6050
      else
      {
        const char err[] = "[MPU] Read FAILED\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t *)err, sizeof(err) - 1, 100);
      } // end else
    } // end if mpu ready

    // BQ
    HAL_StatusTypeDef bq_status;

    bq_status = HAL_I2C_IsDeviceReady(&hi2c2, BQ76942_ADDR, 3, 100);

    char bq_msg[128];

    snprintf(bq_msg, sizeof(bq_msg), "[BQ76942] status=%d, I2C_error=0x%08lX, I2C_state=%d\r\n", (int)bq_status, (unsigned long)HAL_I2C_GetError(&hi2c2), (int)HAL_I2C_GetState(&hi2c2));

    HAL_UART_Transmit(&huart3, (uint8_t *)bq_msg, strlen(bq_msg), 100);

    // delay after all
    HAL_Delay(1000);
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
  hi2c2.Init.Timing = 0x20404768;
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
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x20404768;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  htim6.Init.Period = 561;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED_PG0_Pin|LED_PG1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_PE7_GPIO_Port, LED_PE7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_PG0_Pin LED_PG1_Pin */
  GPIO_InitStruct.Pin = LED_PG0_Pin|LED_PG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_PE7_Pin */
  GPIO_InitStruct.Pin = LED_PE7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PE7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_EN_Pin */
  GPIO_InitStruct.Pin = SD_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_EN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
