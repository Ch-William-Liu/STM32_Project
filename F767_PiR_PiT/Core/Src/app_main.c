#include "app_main.h"
#include "ds3231.h"
#include "relay_control.h"
#include "rtc_scheduler.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart3;

#define RTC_POLL_INTERVAL_MS    1000U
#define SIM_TASK_DURATION_MS    10000U
#define SIM_PIT_OFF_DELAY_MS    2000U

typedef enum
{
  APP_IDLE = 0,
  APP_SIM_TASK,
  APP_SIM_SHUTDOWN
} App_State_t;

