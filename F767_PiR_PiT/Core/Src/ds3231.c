#include "ds3231.h"

extern I2C_HandleTypeDef hi2c2;

#define DS3231_ADDRESS    (0x68U << 1)
#define DS3231_TIME_REG   (0x00U)

static uint8_t BCD_To_DEC(uint8_t value)
{
  return (uint8_t)(((value >> 4) * 10U) + (value & 0x0FU));
} // end function BCD_To_DEC

HAL_StatusTypeDef DS3231_Init(void)
{
  return HAL_I2C_IsDeviceReady(&hi2c2, DS3231_ADDRESS, 3, 100);
} // end function DS3231_Init

HAL_StatusTypeDef DS3231_GetTime(DS3231_Time_t * time)
{
  uint8_t data[7];

  if (time == NULL)
  {
    return HAL_ERROR;
  } // end if time is NULL

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDRESS, DS3231_TIME_REG, I2C_MEMADD_SIZE_8BIT, data, sizeof(data), 100);

  if (status != HAL_OK)
  {
    return status;
  } // end if status not ok

  time->second = BCD_To_DEC(data[0] & 0x7FU);
  time->minute = BCD_To_DEC(data[1] & 0x7FU);

  if (data[2] & 0x40U)
  {
    uint8_t hour12 = BCD_To_DEC(data[2] & 0x1FU);
    time->hour = (uint8_t)((hour12 % 12U) + ((data[2] & 0x20U) ? 12U : 0U));
  } // end if 12-hour system
  else
  {
    time->hour = BCD_To_DEC(data[2] & 0x3FU);
  } // end else
  
  time->weekday = BCD_To_DEC(data[3] & 0x07U);
  time->day = BCD_To_DEC(data[4] & 0x3FU);
  time->month = BCD_To_DEC(data[5] & 0x1FU);

  time->year = (uint16_t)(2000U + BCD_To_DEC(data[6]) + ((data[5] & 0x80U) ? 100U : 0U));

  // check if time is valid or not
  if (time->second > 59U || time->minute > 59U || time->hour > 23U || time->month < 1U || time->month > 12U || time->day < 1U || time->day > 31U)
  {
    return HAL_ERROR;
  } // end if 

  return HAL_OK;
} // end function DS3231_GetTime