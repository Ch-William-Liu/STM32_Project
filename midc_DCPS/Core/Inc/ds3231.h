#ifndef __DS3231_H
#define __DS3231_H

#include "main.h"
#include <stdint.h>

typedef struct
{
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;
}DS3231_Time_t;

HAL_StatusTypeDef DS3231_GetTime(DS3231_Time_t *t);

#endif
