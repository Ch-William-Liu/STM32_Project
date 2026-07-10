#ifndef INC_DS3231_H_
#define INC_DS3231_H_

#include "stm32f7xx_hal.h"
#include <stdint.h>

#define DS3231_I2C_ADDRESS      (0x68<<1U)

#define DS3231_REG_SECONDS       0x00U
#define DS3231_REG_MINUTES       0x01U
#define DS3231_REG_HOURS         0x02U
#define DS3231_REG_DAY           0x03U
#define DS3231_REG_DATE          0x04U
#define DS3231_REG_MONTH         0x05U
#define DS3231_REG_YEAR          0x06U

typedef struct
{
  uint16_t year;
  uint16_t month;
  uint16_t date;
  uint16_t dayOfWeek;
  uint16_t hour;
  uint16_t minute;
  uint16_t second;
} DS3231_DateTime_t ;

HAL_StatusTypeDef DS3231_IsReady(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef DS3231_SetDateTime(I2C_HandleTypeDef *hi2c, const DS3231_DateTime_t *dateTime);

HAL_StatusTypeDef DS3231_GetDateTime(I2C_HandleTypeDef *hi2c, DS3231_DateTime_t *dateTime);

uint8_t DS3231_IsDateTimeValid(const DS3231_DateTime_t *dateTime);

uint8_t DS3231_CalculateDayOfWeek(uint16_t year, uint8_t month, uint8_t date);

#endif
