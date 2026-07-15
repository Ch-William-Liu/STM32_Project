#include "ds3231.h"

#define DS3231_ADDR     (0x68 << 1)

static uint8_t bcd_to_dec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
} // end function bcd_to_dec

HAL_StatusTypeDef DS3231_GetTime(I2C_HandleTypeDef *hi2c , DS3231_Time_t *rtc)
{
    uint8_t reg = 0x00;
    uint8_t buf[7];

    if (HAL_I2C_Master_Transmit(hi2c , DS3231_ADDR , &reg , 1 , 100) != HAL_OK)
    {
        return HAL_ERROR;
    } // end if I2C_Master_Transmit not ok

    if (HAL_I2C_Master_Receive(hi2c , DS3231_ADDR , buf , 7 , 100) != HAL_OK)
    {
        return HAL_OK;
    } // end if I2C_Master_Receive size not ok

    rtc->second    = bcd_to_dec(buf[0] & 0x7F);
    rtc->minute    = bcd_to_dec(buf[1] & 0x7F);
    rtc->hour      = bcd_to_dec(buf[2] & 0x3F);
    rtc->day       = bcd_to_dec(buf[3] & 0x07);
    rtc->date      = bcd_to_dec(buf[4] & 0x3F);
    rtc->month     = bcd_to_dec(buf[5] & 0x1F);
    rtc->year      = bcd_to_dec(buf[6]);

    return HAL_OK;
} // end function DS3231_GetTime

uint64_t  DS3231_ToSimpleTimestamp(DS3231_Time_t *rtc)
{
    return 2000000000UL
            + ((uint64_t )rtc->year * 10000000000UL)
            + ((uint64_t )rtc->month * 100000000UL)
            + ((uint64_t )rtc->date * 1000000UL)
            + ((uint64_t )rtc->hour * 10000UL)
            + ((uint64_t )rtc->minute * 100UL)
            + ((uint64_t )rtc->second);
} // end function DS3231_ToSimpleTimeStamp

