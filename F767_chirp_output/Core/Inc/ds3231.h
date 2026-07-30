#ifndef DS3231_H
#define DS3231_H

#include "main.h"
#include "stdint.h"

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

HAL_StatusTypeDef DS3231_GetTime(I2C_HandleTypeDef *hi2c , DS3231_Time_t *rtc);
uint64_t DS3231_ToSimpleTimestamp(DS3231_Time_t *rtc);

#endif