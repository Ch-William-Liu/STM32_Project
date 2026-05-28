/* USER CODE BEGIN Header */
/* F767_filter_correlation */
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "fatfs.h"

#include "app_config.h"
#include "app_wav.h"
#include "app_chirp.h"
#include "app_detector.h"
#include "app_utils.h"
#include "sd_spi.h"
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
static APP_WavFile wav;

static int16_t rawFrame[APP_FRAME_LEN];
// static int16_t newSamples[APP_HOP_LEN];

static float frameFloat[APP_FRAME_LEN];
static float chirpTemplate[APP_CHIRP_LEN];
static float corrBuffer[APP_CORR_LEN];

static uint32_t frameID = 0;
static uint32_t globalFrameStartSample = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static int APP_ReadFirstFrame(void);
static int APP_ReadNextOverlapFrame(void);
static void APP_ProcessFrame(void);
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
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit(&huart3, (uint8_t *)"HAL UART OK \r\n", 13, HAL_MAX_DELAY);

  BYTE testSector[512];

  DRESULT r = SD_SPI_read(0, testSector, 0, 1);
  printf("Read sector0 result = %d\r\n", r);
  printf("Sector0 end = %02X %02X\r\n", testSector[510], testSector[511]);

  FRESULT fres = f_mount(&USERFatFS, USERPath, 1);

  if (fres != FR_OK)
  {
    printf("FatFs mount failed: %d\r\n", fres);
    Error_Handler();
  } // end if fres ok

  printf("FatFs mount OK\r\n");

  printf("Opening WAV file: %s\r\n", APP_WAV_FILENAME);

  int wavRet = APP_Wav_Open(&wav, APP_WAV_FILENAME);

  if (wavRet != 0)
  {
    printf("WAV open failed: %d\r\n", wavRet);
    Error_Handler();
  } // end if wavRet != 0

  printf("WAV open OK \r\n");
  printf("Sample rate: %lu Hz\r\n", wav.sampleRate);
  printf("Channels: %u\r\n", wav.numChannels);
  printf("Bits: %u\r\n", wav.bitsPerSample);

  if (wav.sampleRate != APP_FS_HZ)
  {
    printf("Warning: WAV sample rate does not match APP_FS_HZ\r\n");
  } // end if sampleRate not the same

  APP_Chirp_Generate(chirpTemplate, APP_CHIRP_LEN);

  printf("Chirp template generated.\r\n");

  int firstRead = APP_ReadFirstFrame();

  if (firstRead <= 0)
  {
    printf("Failed to read first frame\r\n");
    Error_Handler();
  } // end if firstRead <= 0

  printf("First frame read OK\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t t0 = HAL_GetTick();
    APP_ProcessFrame();

    uint32_t t1 = HAL_GetTick();

    printf("Process time: %lu ms\r\n", t1 - t0);
    int readStatus = APP_ReadNextOverlapFrame();

    if (readStatus == 0)
    {
      printf("End of WAV file\r\n");

      APP_Wav_Close(&wav);

      while (1)
      {
        HAL_Delay(1000);
      } // end inner while
      
    } // end if readStatus == 0
    else if (readStatus < 0)
    {
      printf("WAV read error\r\n");
      Error_Handler();
    } // end ekse if
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
} // end function _write

static int APP_ReadFirstFrame(void)
{
  uint32_t samplesRead = 0;

  int ret = APP_Wav_ReadSamples(&wav, rawFrame, APP_FRAME_LEN, &samplesRead);

  if (ret != 0)
  {
    return -1;
  } // end if

  if (samplesRead < APP_FRAME_LEN)
  {
    return 0;
  } // end if

  return 1;
} // end function APP_ReadFirstFrame

static int APP_ReadNextOverlapFrame(void)
{
  uint32_t samplesRead = 0;

  memmove(rawFrame, &rawFrame[APP_HOP_LEN], APP_OVERLAP_LEN * sizeof(int16_t));

  int ret = APP_Wav_ReadSamples(&wav, &rawFrame[APP_OVERLAP_LEN], APP_HOP_LEN, &samplesRead);

  if (ret != 0)
  {
    return -1;
  } // end if

  if (samplesRead < APP_HOP_LEN)
  {
    return 0;
  } // end if

  globalFrameStartSample += APP_HOP_LEN;

  return 1;
} // end function APP_ReadNextOverlapFrame

static void APP_ProcessFrame(void)
{
  APP_Int16ToFloat(rawFrame, frameFloat, APP_FRAME_LEN);

  APP_RemoveDC(frameFloat, APP_FRAME_LEN);

  APP_NormalizeFrame(frameFloat, APP_FRAME_LEN);

  APP_Detector_Correlate(frameFloat, chirpTemplate, corrBuffer);

  APP_DetectroResult result = APP_Detector_FindPeak(corrBuffer, APP_CORR_LEN, APP_DETECH_TH);

  uint32_t detectedSample = globalFrameStartSample + result.peakIndex;
  float detectTime = (float)detectedSample / (float)APP_FS_HZ;

  printf("Frame %lu | Peak %.4f | LocalIndex %lu | Sample %lu | Time %.4f s | %s\r\n", frameID , result.peakValue, result.peakIndex, detectedSample, detectTime, result.detected ? "DETECTED" : "NONE");

  frameID ++;
}
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
