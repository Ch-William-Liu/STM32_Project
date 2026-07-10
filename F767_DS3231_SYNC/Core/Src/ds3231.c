#include "ds3231.h"

#define DS3231_I2C_TIMEOUT_MS    100U

static uint8_t DS3231_DecimalToBCD(uint8_t value)
{
  return (uint8_t)(((value / 10U) << 4U) | (value % 10U));
} // end function DS3231_DecimalToBCD

static uint8_t DS3231_BCDToDecimal(uint8_t value)
{
  return (uint8_t)(((value >> 4U) * 10U) | (value & 0x0F));
} // end function DS3231_BCDToDecimal

static uint8_t DS3231_IsLeapYear(uint16_t year)
{
  if ((year % 400U) == 0U)
  {
    return 1U;
  } // end if 
  
  if ((year % 100U) == 0U)
  {
    return 0U;
  } // end if

  if ((year % 4U) == 0U)
  {
    return 1U;
  } // end if 

  return 0U;
} // end function DS3231_IsLeapYear

static uint8_t DS3231_GetDaysInMonth(uint16_t year, uint8_t month)
{
  static const uint8_t daysInMonth[12] = 
  {
    31U , 28U , 31U , 30U ,
    31U , 30U , 31U , 31U ,
    30U , 31U , 30U , 31U
  };

  if ((month < 1U) || (month > 12U))
  {
    return 0U;
  } // end if invalid month

  if ((month == 2U) && DS3231_IsLeapYear(year))
  {
    return 29U;
  } // end if leap year

  return daysInMonth[month - 1U];
} // end function DS3231_GetDaysInMonth

HAL_StatusTypeDef DS3231_IsReady(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == NULL)
  {
    return HAL_ERROR;
  } // end if i2c is null

  return HAL_I2C_IsDeviceReady(hi2c , DS3231_I2C_ADDRESS , 3U , DS3231_I2C_TIMEOUT_MS);
} // end function DS3231_IsReady

uint8_t DS3231_IsDateTimeValid(const DS3231_DateTime_t *dateTime)
{
  uint8_t maximumDate;

  if (dateTime == NULL)
  {
    return 0U;
  } // end if dateTime is NULL

  if ((dateTime->year < 2000U) || (dateTime->year > 2099U))
  {
    return 0U;
  } // end if year is invalid

  if ((dateTime->month < 1U) || (dateTime->month > 12U))
  {
    return 0U;
  } // end if month is invalid

  maximumDate = DS3231_GetDaysInMonth(dateTime->year , dateTime->month);

  if ((dateTime->date < 1U) || (dateTime->date > maximumDate))
  {
    return 0U;
  } // end if data is invalid

  if (dateTime->hour > 23U)
  {
    return 0U;
  } // end if hour is invalid

  if (dateTime->minute > 59U)
  {
    return 0U;
  } // end if minute is invalid

  if (dateTime->second > 59U)
  {
    return 0U;
  } // end if second is invalid

  return 1U;
} // end function DS3231_IsDateTimeValid

uint8_t DS3231_CalculateDayOfWeek(uint16_t year, uint8_t month, uint8_t date)
{
  static const uint8_t monthTable[12] =
  {
    0U, 3U, 2U, 5U, 0U, 3U,
    5U, 1U, 4U, 6U, 2U, 4U
  };

  uint16_t calculationYear = year;
  uint8_t day;

  if (month < 3U)
  {
    calculationYear--;
  } // end if month less than 3

    day = (uint8_t)(
      (
          calculationYear
          + calculationYear / 4U
          - calculationYear / 100U
          + calculationYear / 400U
          + monthTable[month - 1U]
          + date
      ) % 7U
  );

  return (uint8_t)(day + 1U);
} // end function DS3231_CalculateDayOfWeek

HAL_StatusTypeDef DS3231_SetDateTime(I2C_HandleTypeDef *hi2c, const DS3231_DateTime_t *dateTime)
{
  uint8_t data[7];
  uint8_t dayOfweek;

  if ((hi2c == NULL) || (dateTime == NULL))
  {
    return HAL_ERROR;
  } // end if i2c is null

  if (!DS3231_IsDateTimeValid(dateTime))
  {
    return HAL_ERROR;
  } // end if time is invalid

  dayOfweek = dateTime->dayOfWeek;

  if ((dayOfweek < 1U) || (dayOfweek > 7U))
  {
    dayOfweek = DS3231_CalculateDayOfWeek(dateTime->year , dateTime->month , dateTime->date);
  } // end if dayOfweek is invalid

  data[0] = DS3231_DecimalToBCD(dateTime->second);
  data[1] = DS3231_DecimalToBCD(dateTime->minute);

  data[2] = DS3231_DecimalToBCD(dateTime->hour);
  data[3] = DS3231_DecimalToBCD(dayOfweek);
  data[4] = DS3231_DecimalToBCD(dateTime->date);

  data[5] = DS3231_DecimalToBCD(dateTime->month);
  data[6] = DS3231_DecimalToBCD((uint8_t)(dateTime->year - 2000U));

  return HAL_I2C_Mem_Write(hi2c , DS3231_I2C_ADDRESS , DS3231_REG_SECONDS , I2C_MEMADD_SIZE_8BIT , data , sizeof(data) , DS3231_I2C_TIMEOUT_MS);
} // end function DS3231_SetDateTime

HAL_StatusTypeDef DS3231_GetDateTime(I2C_HandleTypeDef *hi2c, DS3231_DateTime_t *dateTime)
{
  uint8_t data[7];
  HAL_StatusTypeDef status;

  if ((hi2c == NULL) || (dateTime == NULL))
  {
    return HAL_ERROR;
  } // end if hi2c is invalid

  status = HAL_I2C_Mem_Read(hi2c , DS3231_I2C_ADDRESS , DS3231_REG_SECONDS , I2C_MEMADD_SIZE_8BIT , data , sizeof(data) , DS3231_I2C_TIMEOUT_MS);

  if (status != HAL_OK)
  {
    return status;
  } // end if status not ok

  dateTime->second = DS3231_BCDToDecimal(data[0] & 0x7FU);
  dateTime->minute = DS3231_BCDToDecimal(data[1] & 0x7FU);
  
  if ((data[2] & 0x40U) != 0U)
  {
    uint8_t hour12;
    uint8_t isPM;

    hour12 = DS3231_BCDToDecimal(data[2] & 0x1FU);

    isPM = (data[2] & 0x20U) ? 1U : 0U;

    if (hour12 == 12U)
    {
      dateTime->hour = isPM ? 12U : 0U;
    } // end if 
    else
    {
      dateTime->hour = isPM ? (uint8_t)(hour12 + 12U) : hour12;
    } // end else
  } // end if 
  else
  {
    dateTime->hour = DS3231_BCDToDecimal(data[2] & 0x3FU);
  } // end else

  dateTime->dayOfWeek = DS3231_BCDToDecimal(data[3] & 0x07U);

  dateTime->date = DS3231_BCDToDecimal(data[4] & 0x3FU);

  dateTime->month = DS3231_BCDToDecimal(data[5] & 0x1FU);

  dateTime->year = (uint16_t)(2000U + DS3231_BCDToDecimal(data[6]));

  if (!DS3231_IsDateTimeValid(dateTime))
  {
    return HAL_ERROR;
  } // end if 

  return HAL_OK;
} // end function DS3231_GetDateTime