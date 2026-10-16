/* USER CODE BEGIN Header */
/* F767_ProjectHarbour */
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
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t second;
  uint8_t minute;
  uint8_t hour;
  uint8_t day;
  uint8_t date;
  uint8_t month;
  uint8_t year;
} DS3231_Time_t;

typedef enum
{
  SYS_IDLE = 0,
  SYS_AMP_STARTUP,
  SYS_PI_BOOTING,
  SYS_WAIT_TX_DONE,
  SYS_WAIT_SAFE_OFF,
  SYS_PI_POWER_OFF_DELAY,
  SYS_ERROR
} SystemState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DS3231_ADDR             (0x68 << 1)

#define SCHEDULE_INTERVAL_MIN   5

#define PI_BOOT_TIMEOUT_MS      90000UL
#define TX_TIMEOUT_MS           180000UL
#define SHUTDOWN_TIMEOUT_MS     60000UL

#define READY_CONFIRM_MS        500UL
#define DONE_CONFIRM_MS         1000UL
#define SAFE_OFF_CONFIRM_MS     200UL

#define AMP_WARMUP_MS           2000UL
#define ERROR_RECOVERY_MS       5000UL
#define RTC_POLL_INTERVAL_MS    100UL
#define PI_POWER_OFF_DELAY_MS   10000UL

/* Only trigger near the beginning of the scheduled minute. */
#define SCHEDULE_SECOND_WINDOW  1U

#define RELAY_ACTIVE_LOW        1
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

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
DS3231_Time_t rtc_time;

SystemState_t system_state = SYS_IDLE;

uint32_t state_start_tick = 0;

uint32_t ready_high_tick = 0;
uint32_t done_high_tick = 0;
uint32_t safe_high_tick = 0;
uint32_t last_rtc_poll_tick = 0;

uint32_t last_trigger_id = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ============================================================
// === DS3231 + Raspberry Pi / Amplifier supervisor functions
// ============================================================
static uint8_t BCD_To_Dec(uint8_t value)
{
  return ((value >> 4) * 10U) + (value & 0x0FU);
}

HAL_StatusTypeDef DS3231_ReadTime(DS3231_Time_t *time)
{
  uint8_t data[7];

  if (HAL_I2C_Mem_Read(&hi2c2,
                       DS3231_ADDR,
                       0x00,
                       I2C_MEMADD_SIZE_8BIT,
                       data,
                       7,
                       100) != HAL_OK)
  {
    return HAL_ERROR;
  }

  time->second = BCD_To_Dec(data[0] & 0x7F);
  time->minute = BCD_To_Dec(data[1] & 0x7F);
  time->hour   = BCD_To_Dec(data[2] & 0x3F);   // DS3231 configured for 24-hour mode
  time->day    = BCD_To_Dec(data[3] & 0x07);
  time->date   = BCD_To_Dec(data[4] & 0x3F);
  time->month  = BCD_To_Dec(data[5] & 0x1F);
  time->year   = BCD_To_Dec(data[6]);

  return HAL_OK;
}

// ============================================================
// === Relay control
// === RELAY_ACTIVE_LOW = 1 means LOW -> relay ON
// ============================================================
void Pi_Relay_On(void)
{
#if RELAY_ACTIVE_LOW
  HAL_GPIO_WritePin(PI_RELAY_GPIO_Port, PI_RELAY_Pin, GPIO_PIN_RESET);
#else
  HAL_GPIO_WritePin(PI_RELAY_GPIO_Port, PI_RELAY_Pin, GPIO_PIN_SET);
#endif
}

void Pi_Relay_Off(void)
{
#if RELAY_ACTIVE_LOW
  HAL_GPIO_WritePin(PI_RELAY_GPIO_Port, PI_RELAY_Pin, GPIO_PIN_SET);
#else
  HAL_GPIO_WritePin(PI_RELAY_GPIO_Port, PI_RELAY_Pin, GPIO_PIN_RESET);
#endif
}

void Amp_Relay_On(void)
{
#if RELAY_ACTIVE_LOW
  HAL_GPIO_WritePin(AMP_RELAY_GPIO_Port, AMP_RELAY_Pin, GPIO_PIN_RESET);
#else
  HAL_GPIO_WritePin(AMP_RELAY_GPIO_Port, AMP_RELAY_Pin, GPIO_PIN_SET);
#endif
}

void Amp_Relay_Off(void)
{
#if RELAY_ACTIVE_LOW
  HAL_GPIO_WritePin(AMP_RELAY_GPIO_Port, AMP_RELAY_Pin, GPIO_PIN_SET);
#else
  HAL_GPIO_WritePin(AMP_RELAY_GPIO_Port, AMP_RELAY_Pin, GPIO_PIN_RESET);
#endif
}

// ============================================================
// === STM32 -> Raspberry Pi TX enable
// ============================================================
void Tx_Enable_On(void)
{
  HAL_GPIO_WritePin(TX_ENABLE_GPIO_Port, TX_ENABLE_Pin, GPIO_PIN_SET);
}

void Tx_Enable_Off(void)
{
  HAL_GPIO_WritePin(TX_ENABLE_GPIO_Port, TX_ENABLE_Pin, GPIO_PIN_RESET);
}

// ============================================================
// === Schedule helper
// === Unique ID for YY/MM/DD HH:MM, used to prevent duplicate trigger
// ============================================================
static uint32_t Get_Schedule_ID(const DS3231_Time_t *t)
{
  /*
   * Pack YY/MM/DD HH:MM into 27 bits:
   * year(7) + month(4) + date(5) + hour(5) + minute(6).
   * This avoids decimal multiplication overflow on uint32_t.
   */
  return (((uint32_t)t->year  & 0x7FU) << 20) |
         (((uint32_t)t->month & 0x0FU) << 16) |
         (((uint32_t)t->date  & 0x1FU) << 11) |
         (((uint32_t)t->hour  & 0x1FU) <<  6) |
         (((uint32_t)t->minute & 0x3FU));
}

static bool Is_Schedule_Time(const DS3231_Time_t *t)
{
  uint32_t trigger_id;

  if ((t->minute % SCHEDULE_INTERVAL_MIN) != 0U)
  {
    return false;
  }

  /* Trigger only at the beginning of 00, 05, 10, ... minutes. */
  if (t->second > SCHEDULE_SECOND_WINDOW)
  {
    return false;
  }

  trigger_id = Get_Schedule_ID(t);

  if (trigger_id == last_trigger_id)
  {
    return false;
  }

  return true;
}

// ============================================================
// === GPIO long-high confirmation / glitch rejection
// ============================================================
static bool Confirm_High(GPIO_TypeDef *port,
                         uint16_t pin,
                         uint32_t *high_start_tick,
                         uint32_t confirm_ms)
{
  uint32_t now = HAL_GetTick();

  if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
  {
    if (*high_start_tick == 0U)
    {
      /* Avoid using 0 as a valid start marker. */
      *high_start_tick = (now == 0U) ? 1U : now;
    }

    if ((now - *high_start_tick) >= confirm_ms)
    {
      return true;
    }
  }
  else
  {
    *high_start_tick = 0U;
  }

  return false;
}

// ============================================================
// === Autonomous system state machine
// ============================================================
static void System_StateMachine(void)
{
  uint32_t now = HAL_GetTick();

  switch (system_state)
  {
    case SYS_IDLE:
      /* Safe idle outputs. */
      Tx_Enable_Off();
      Amp_Relay_Off();
      Pi_Relay_Off();

      /* Poll DS3231 at 10 Hz instead of on every main-loop iteration. */
      if ((now - last_rtc_poll_tick) >= RTC_POLL_INTERVAL_MS)
      {
        last_rtc_poll_tick = now;

        if (DS3231_ReadTime(&rtc_time) == HAL_OK)
        {
          if (Is_Schedule_Time(&rtc_time))
          {
            last_trigger_id = Get_Schedule_ID(&rtc_time);

            ready_high_tick = 0U;
            done_high_tick = 0U;
            safe_high_tick = 0U;

            /* Step 1: power amplifier first. */
            Amp_Relay_On();

            state_start_tick = now;
            system_state = SYS_AMP_STARTUP;
          }
        }
      }
      break;

    case SYS_AMP_STARTUP:
      /* Step 2: wait for amplifier power to stabilize before powering Raspberry Pi. */
      if ((now - state_start_tick) >= AMP_WARMUP_MS)
      {
        Pi_Relay_On();

        ready_high_tick = 0U;
        safe_high_tick = 0U;
        state_start_tick = now;
        system_state = SYS_PI_BOOTING;
      }
      break;

    case SYS_PI_BOOTING:
      /* Pi raises PI_READY after GPS fix + waveform preparation. */
      if (Confirm_High(PI_READY_GPIO_Port,
                       PI_READY_Pin,
                       &ready_high_tick,
                       READY_CONFIRM_MS))
      {
        /* Step 3: amplifier is already on; allow acoustic transmission now. */
        Tx_Enable_On();
        done_high_tick = 0U;
        safe_high_tick = 0U;
        state_start_tick = now;
        system_state = SYS_WAIT_TX_DONE;
      }
      else if (Confirm_High(PI_SAFE_OFF_GPIO_Port,
                            PI_SAFE_OFF_Pin,
                            &safe_high_tick,
                            SAFE_OFF_CONFIRM_MS))
      {
        /* Python failed before READY and requested shutdown. */
        Tx_Enable_Off();
        Amp_Relay_Off();
        state_start_tick = now;
        system_state = SYS_PI_POWER_OFF_DELAY;
      }
      else if ((now - state_start_tick) >= PI_BOOT_TIMEOUT_MS)
      {
        /* Pi never became ready: fail safely. */
        Tx_Enable_Off();
        Amp_Relay_Off();
        Pi_Relay_Off();
        state_start_tick = now;
        system_state = SYS_ERROR;
      }
      break;

    case SYS_WAIT_TX_DONE:
      /* Pi holds PI_TX_DONE high for several seconds after playback ends. */
      if (Confirm_High(PI_TX_DONE_GPIO_Port,
                       PI_TX_DONE_Pin,
                       &done_high_tick,
                       DONE_CONFIRM_MS))
      {
        /* Step 4: stop TX permission and remove amplifier power immediately. */
        Tx_Enable_Off();
        Amp_Relay_Off();

        safe_high_tick = 0U;
        state_start_tick = now;
        system_state = SYS_WAIT_SAFE_OFF;
      }
      else if (Confirm_High(PI_SAFE_OFF_GPIO_Port,
                            PI_SAFE_OFF_Pin,
                            &safe_high_tick,
                            SAFE_OFF_CONFIRM_MS))
      {
        /* Python failed during TX stage and requested shutdown. */
        Tx_Enable_Off();
        Amp_Relay_Off();
        state_start_tick = now;
        system_state = SYS_PI_POWER_OFF_DELAY;
      }
      else if ((now - state_start_tick) >= TX_TIMEOUT_MS)
      {
        /* Transmission hung or no DONE received. */
        Tx_Enable_Off();
        Amp_Relay_Off();

        /* Give Linux a chance to power off; force-cut later if it never does. */
        safe_high_tick = 0U;
        state_start_tick = now;
        system_state = SYS_WAIT_SAFE_OFF;
      }
      break;

    case SYS_WAIT_SAFE_OFF:
      /* Python raises PI_SAFE_OFF before Linux shutdown; wait for confirmation. */
      if (Confirm_High(PI_SAFE_OFF_GPIO_Port,
                       PI_SAFE_OFF_Pin,
                       &safe_high_tick,
                       SAFE_OFF_CONFIRM_MS))
      {
        /* Step 5: begin the non-blocking delay before removing physical Pi power. */

        state_start_tick = now;

        system_state = SYS_PI_POWER_OFF_DELAY;
      }
      else if ((now - state_start_tick) >= SHUTDOWN_TIMEOUT_MS)
      {
        /* Last-resort fail-safe: force power removal if Pi never signals safe-off. */
        Tx_Enable_Off();
        Amp_Relay_Off();
        Pi_Relay_Off();
        state_start_tick = now;
        system_state = SYS_ERROR;
      }
      break;

    case SYS_PI_POWER_OFF_DELAY:
      if ((now - state_start_tick) >= PI_POWER_OFF_DELAY_MS)
      {
          Pi_Relay_Off();

          printf("[STM32] PI_RELAY -> OFF\r\n");

          system_state = SYS_IDLE;
      }

      break;

    case SYS_ERROR:
      /* Always leave the hardware in a safe de-energized state. */
      Tx_Enable_Off();
      Amp_Relay_Off();
      Pi_Relay_Off();

      if ((now - state_start_tick) >= ERROR_RECOVERY_MS)
      {
        ready_high_tick = 0U;
        done_high_tick = 0U;
        safe_high_tick = 0U;
        system_state = SYS_IDLE;
      }
      break;

    default:
      Tx_Enable_Off();
      Amp_Relay_Off();
      Pi_Relay_Off();
      state_start_tick = now;
      system_state = SYS_ERROR;
      break;
  }
}
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
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  // ============================================================
  // === Force a known-safe startup state
  // ============================================================
  Tx_Enable_Off();
  Amp_Relay_Off();
  Pi_Relay_Off();

  ready_high_tick = 0U;
  done_high_tick = 0U;
  safe_high_tick = 0U;
  last_rtc_poll_tick = HAL_GetTick();
  system_state = SYS_IDLE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    System_StateMachine();
    HAL_Delay(10);
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
  HAL_GPIO_WritePin(GPIOC, PI_RELAY_Pin|AMP_RELAY_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TX_ENABLE_GPIO_Port, TX_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PI_RELAY_Pin AMP_RELAY_Pin TX_ENABLE_Pin */
  GPIO_InitStruct.Pin = PI_RELAY_Pin|AMP_RELAY_Pin|TX_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PI_TX_DONE_Pin PI_SAFE_OFF_Pin */
  GPIO_InitStruct.Pin = PI_TX_DONE_Pin|PI_SAFE_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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

  /*Configure GPIO pin : PI_READY_Pin */
  GPIO_InitStruct.Pin = PI_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PI_READY_GPIO_Port, &GPIO_InitStruct);

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
