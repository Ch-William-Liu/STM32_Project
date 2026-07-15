#ifndef INC_DS3231_H__
#define INC_DS3231_H__

#include "stm32f7xx_hal.h"
#include <stdint.h>

#define DS3231_ADDRESS      (0x68U << 1U)

typedef struct
{
  uint16_t year;
  uint8_t month;
  uint8_t date;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} DS3231_Datetime_t;

HAL_StatusTypeDef DS3231_IsReady(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef DS3231_SetDateTime(I2C_HandleTypeDef *hi2c, const DS3231_Datetime_t *dateTime);
HAL_StatusTypeDef DS3231_GetDataTime(I2C_HandleTypeDef *hi2c, const DS3231_Datetime_t *dateTime);

uint8_t DS3231_CalculateDay(uint16_t year, uint8_t month, uint8_t date);

#endif
