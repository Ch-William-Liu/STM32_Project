#include "ds3231.h"
#include <stdint.h>

#define DS3231_TIMEOUT_MS     100U

static uint8_t DecimalToBCD(uint8_t value)
{
  return (uint8_t)((value / 10U) << 4U | (value % 10U)); 
} // end function DecimalToBCD

static uint8_t BCDToDecimal(uint8_t value)
{
  return (uint8_t)((value >> 4U) * 10U + (value & 0x0FU));
} // end function BCDToDecimal

HAL_StatusTypeDef DS3231_IsReady(I2C_HandleTypeDef *hi2c)
{
  return HAL_I2C_IsDeviceReady(hi2c , DS3231_ADDRESS , 3U , DS3231_TIMEOUT_MS);
} // end function DS3231_IsReady

uint8_t DS3231_CalculateDay(uint16_t year, uint8_t month, uint8_t date)
{
  static const uint8_t monthTable[12] = {
    0U, 3U, 2U, 5U, 0U, 3U,
    5U, 1U, 4U, 6U, 2U, 4U
  };

  uint16_t y = year;
  uint8_t result;

  if ((month < 1U) || (month > 12U))
  {
    return 1U;
  } // end if month is invalid

  if (month < 3U)
  {
    y--;
  } // end if month less than 3

  result = ((uint8_t)(y + y / 4U - y / 100U + y / 400U + monthTable[month - 1U] + date) % 7U);

  return (uint8_t)(result + 1U);
} // end function DS3231_CalculateDay

HAL_StatusTypeDef DS3231_SetDateTime(I2C_HandleTypeDef *hi2c, const DS3231_Datetime_t *dateTime)
{
  uint8_t data[7];

  if ((hi2c == NULL) || (dateTime == NULL))
  {
    return HAL_ERROR;
  } // end if i2c is empty and dateTime

  if (
    (dateTime->year < 2000U) || (dateTime->year > 2099U) ||
    (dateTime->month < 1U) || (dateTime->month > 12U) ||
    (dateTime->date < 1U) || (dateTime->date > 31U) ||
    (dateTime->hour > 23U) || (dateTime->minute > 59U) || (dateTime->second > 59U))
  {
    return HAL_ERROR;
  } // end if time is invalid


  data[0] = DecimalToBCD(dateTime->second);
  data[1] = DecimalToBCD(dateTime->minute);
  data[2] = DecimalToBCD(dateTime->hour); // in 24-hour system
  data[3] = DecimalToBCD(dateTime->day);
  data[4] = DecimalToBCD(dateTime->date);
  data[5] = DecimalToBCD(dateTime->month);
  data[6] = DecimalToBCD((uint8_t)(dateTime->year - 2000U));

  HAL_StatusTypeDef status;
  status = HAL_I2C_Mem_Write(hi2c, DS3231_ADDRESS , 0x00U, I2C_MEMADD_SIZE_8BIT, data , 7U, DS3231_TIMEOUT_MS);

  return status;
} // end function DS3231_SetDateTime

HAL_StatusTypeDef DS3231_GetDataTime(I2C_HandleTypeDef *hi2c, DS3231_Datetime_t *dateTime)
{
  uint8_t data[7];
  HAL_StatusTypeDef status;

  if ((hi2c == NULL) || (dateTime == NULL))
  {
    return HAL_ERROR;
  } // end if i2c and dateTime is NULL

  status = HAL_I2C_Mem_Read(hi2c, DS3231_ADDRESS , 0x00U, I2C_MEMADD_SIZE_8BIT , data, 7U, DS3231_TIMEOUT_MS);

  if (status != HAL_OK)
  {
    return status;
  } // end if mem read not OK

  dateTime->second = BCDToDecimal(data[0] & 0x7FU);
  dateTime->minute = BCDToDecimal(data[1] & 0x7FU);
  dateTime->hour = BCDToDecimal(data[2] & 0x3FU);
  dateTime->day = BCDToDecimal(data[3] & 0x07U);
  dateTime->date = BCDToDecimal(data[4] & 0x3FU);
  dateTime->month = BCDToDecimal(data[5] & 0x3FU);
  dateTime->year = 2000 + BCDToDecimal(data[6]);

  return HAL_OK;
} // end function DS3231_GetDataTime