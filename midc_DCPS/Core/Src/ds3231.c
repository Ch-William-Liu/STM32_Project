#include "ds3231.h"

extern I2C_HandleTypeDef hi2c2;

#define DS3231_ADDR (0x68<<1)

static uint8_t BCD_To_Dec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
} // end function BCD_To_Dec

HAL_StatusTypeDef DS3231_GetTime(DS3231_Time_t *t)
{
    uint8_t data[7];

    if (HAL_I2C_Mem_Read(&hi2c2 , DS3231_ADDR , 0x00 , I2C_MEMADD_SIZE_8BIT , data , 7 , 1000) != HAL_OK)
    {
        return HAL_ERROR;
    } // end if

    t->sec = BCD_To_Dec(data[0] & 0x7F);
    t->min = BCD_To_Dec(data[1] & 0x7F);
    t->hour = BCD_To_Dec(data[2] & 0x3F);
    t->day = BCD_To_Dec(data[3]);
    t->day = BCD_To_Dec(data[4]);
    t->month = BCD_To_Dec(data[5] & 0x1F);
    t->year = BCD_To_Dec(data[6]);

    return HAL_OK;
} // end function DS3231_GetTime