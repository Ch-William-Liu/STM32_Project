/* USER CODE BEGIN Header */
/* F767_Read_MPU6050_NEO6M */
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
#include <stdlib.h>
#include <stdint.h>
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

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;   // GPS
extern UART_HandleTypeDef huart3;   // PuTTy monitor

// USART3 printf redirect
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart3, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
} // end function _write

// MPU6050 (I2C) minial driver
#define MPU_ADDR_7_BIT    0x68
#define MPU_ADDR          (MPU_ADDR_7_BIT << 1)

#define MPU_REG_PWR_MGMT1   0x6B
#define MPU_REG_ACCEL_XOUT  0x3B

typedef struct
{
  int16_t ax , ay , az;
  int16_t gx , gy , gz;
  int16_t temp_raw;
  uint8_t ok;
} mpu6050_t;


static mpu6050_t mpu;

// Initialize MPU6050
static uint8_t mpu6050_init(void)
{
  uint8_t data = 0x00; // wake up
  if (HAL_I2C_Mem_Write(&hi2c1 , MPU_ADDR , MPU_REG_PWR_MGMT1 , 1 , &data , 1 , 100) != HAL_OK)
  {
    mpu.ok = 0;
    return 0;
  } // end if
  mpu.ok = 1;
  return 1;
} // end function mpu6050_init

static uint8_t mpu6050_read14(void)
{
  uint8_t buf[14];
  if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR , MPU_REG_ACCEL_XOUT , 1 , buf , 14 , 100) != HAL_OK)
  {
    mpu.ok = 0;
    return 0;
  } // end if 

  mpu.ax = (int16_t)((buf[0] << 8) | buf[1]);
  mpu.ay = (int16_t)((buf[2] << 8) | buf[3]);
  mpu.az = (int16_t)((buf[4] << 8) | buf[5]);

  mpu.temp_raw = (int16_t)((buf[6] << 8) | buf[7]);

  mpu.gx = (int16_t)((buf[8] << 8) | buf[9]);
  mpu.gy = (int16_t)((buf[10] << 8) | buf[11]);
  mpu.gz = (int16_t)((buf[12] << 8) | buf[13]);

  mpu.ok = 1;
  return 1;
} // end function mpu6050_read14

// Temp = in 0.01 degC (avoid folat printf)
// Temp(degC) = raw / 340 + 36.53
static int32_t mpu6050_temp_c_x100(void)
{
  return (int32_t)mpu.temp_raw * 100 / 340 + 3653;
} // end function mpu6050_temp_c_x100

// GPS (USART2) NMEA RX + parse GGA
// Output lat/lon as int32 in 1e-5 (22.54321 -> 2254321)
typedef struct
{
  uint8_t has_fix;
  uint8_t sats;
  uint8_t lat_e5;   // deg * 1e5
  uint8_t lon_e5;
  uint32_t last_rx_ms;
  uint32_t last_fix_ms;
} gps_t;

static gps_t gps;

static uint8_t gps_rx_byte;
static char gps_line[128];
static uint32_t gps_idx = 0;

static int32_t nmea_degmin_to_e5(const  char *degmin, char hemi)
{
  // degmin: ddmm.mmmm (lat) or dddmm.mmmm (lon)
  // return degrees * 1e5
  if (!degmin || degmin[0] == '\0') return 0;

  double v = atof(degmin);        // safe enough for parsing
  int deg = (int)(v / 100.00);
  double min = v - (double)deg * 100.00;
  double decdeg = (double)deg + (min / 60.00);
  int32_t out = (int32_t)(decdeg * 100000.0 + (decdeg >=0 ? 0.5 : -0.5));

  if (hemi == 'S' || 'W')  out = -out;
  return out;
} // end function nmea_degmin_to_e5

static void gps_parse_gga(const char *line)
{
  // Accept $GPGGA or $GNGGA
  if (strncmp(line, "$GPGGA" , 6) != 0 && strncmp(line , "$GNGGA" , 6) != 0) return;
  
  // Cope to temp buffer then tokenize
  char tmp[128];
  strncpy(tmp, line, sizeof(tmp) - 1);
  tmp[sizeof(tmp) - 1] = '\0';

  // fields:
  // 0=$GxGGA,1=UTC,2=lat,3=N/S,4=lon,5=E/W,6=fix,7=sats,...
  char *save = NULL;
  char *tok = strtok_r(tmp , "," , &save);

  int field = 0;
  const char *lat_s = NULL;
  const char *lon_s = NULL;
  char lat_h = 0, lon_h = 0;
  int fix = 0;
  int sats = 0;

  while (tok)
  {
    if (field == 2) lat_s = tok;
    if (field == 3) lat_h = tok[0];
    if (field == 4) lon_s = tok;
    if (field == 5) lon_h = tok[0];
    if (field == 6) fix = atoi(tok);
    if (field == 7) sats = atoi(tok);

    tok = strtok_r(NULL , "," , &save);
    field++;
  } // end while

  gps.has_fix = (fix > 0) ? 1 : 0;
  gps.sats = (uint8_t)sats;

  if (gps.has_fix && lat_s && lon_s && lat_h && lon_h)
  {
    gps.lat_e5 = nmea_degmin_to_e5(lat_s , lat_h);
    gps.lon_e5 = nmea_degmin_to_e5(lon_s , lon_h);
    gps.last_fix_ms = HAL_GetTick();
  } // end if
} // end function gps_parse_gga

static void gps_start_rx_it(void)
{
  gps.last_rx_ms = HAL_GetTick();
  HAL_UART_Receive_IT(&huart2 , &gps_rx_byte , 1);
} // end function gps_start_rx_it

// USART2 RX interrupt callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    gps.last_rx_ms = HAL_GetTick();

    if (gps_idx < sizeof(gps_line) - 1)
    {
      gps_line[gps_idx++] = (char)gps_rx_byte;
    } // end if

    if (gps_rx_byte == '\n')
    {
      gps_line[gps_idx] = '\0';
      gps_parse_gga(gps_line);
      gps_idx = 0;
    } // end if

    HAL_UART_Receive_IT(&huart2 , &gps_rx_byte , 1);
  } // end if
} // end function HAL_UART_RxCpltCallback

static void i2c_scan_print(void)
{
  printf("I2C scan start...\r\n");
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1) , 2 , 20) == HAL_OK)
    {
      printf("  Found device at 0x%02X.\r\n");
    } // end if
  } // end for
} // end function i2c_scan_print
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n=== Stage 3: MPU6050(I2C) + NEO-6M(USART2) Monitor (USART3 -> PuTTy) === \r\n");
  printf("Print period: 0.5 s | LD2 toggles every print.\r\n");
  i2c_scan_print();
  // Init MPU6050
  if (mpu6050_init())
  {
    printf("MPU6050 init: OK.\r\n");
  } // end if 
  else
  {
    printf("MPU6050 init FAIL: (check I2C wiring/address)\r\n");

    // Start GPS Rx interrupt (USART2)
    gps_start_rx_it();

  } // end else

  // check who am i
  uint8_t who = 0xFF;
  HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR , 0x75 , 1 , &who , 1 , 100);
  printf("MPU WHO_AM_I = 0x%02X\r\n" , who);

  // Start GPS Rx interrupt (USART2)
  gps_start_rx_it();
  printf("GPS RX (USART2) started.\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t last_print_ms = 0;
    uint32_t now = HAL_GetTick();

    if ((now - last_print_ms) >= 500)
    {
      last_print_ms = now;

      // Toggle LD2 (Blue LED) every report
      HAL_GPIO_TogglePin(GPIOB , LD2_Pin);

      // Read MCU
      uint8_t mpu_ok = mpu6050_read14();
      int32_t temp_c_x100 = mpu6050_temp_c_x100();

      // GPS age INFO
      uint32_t gps_rx_age = now - gps.last_rx_ms;
      uint32_t gps_fix_age = now - gps.last_fix_ms;

      // Print
      if (mpu_ok)
      {
        printf("t=%lums | MPU ax=%d ay=%d az=%d gx=%d gy=%d gz=%d temp=%ld.%02ldC |",
        (unsigned long)now,
        mpu.ax, mpu.ay, mpu.az,
        mpu.gx, mpu.gy, mpu.gz,
        (long)(temp_c_x100 / 100),
        (long)labs(temp_c_x100 % 100));
      } // end if

      else
      {
        printf("t=%lums | MPU READ FAIL | ", (unsigned long)now);
      }

      // GPS print
      if (gps_rx_age > 2000)
      {
        printf("GPS: NO DATA (rx_age=%lums).\r\n",(unsigned long)gps_rx_age);
      } // end if
      else if (!gps.has_fix)
      {
        printf("GPS: NO FIX (sats=%u, fix_age=%lums).\r\n",gps.sats,(unsigned long)gps_fix_age);
      } // end else if
      else
      {
        // Print as decimal with 5 digits after decimal without float
        int32_t lat = gps.lat_e5;
        int32_t lon = gps.lon_e5;

        printf("GPS: FIX sats=%u lat=%ld.%05ld lon=%ld.%05ld (fix_age=%lums).\r\n",
        gps.sats,
        (long)(lat / 100000), (long)labs(lat % 100000),
        (long)(lon / 100000), (long)labs(lon % 100000),
        (unsigned long)gps_fix_age);
      } // end else
    } // end if

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
