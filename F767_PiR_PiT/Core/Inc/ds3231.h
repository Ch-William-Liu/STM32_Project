#ifndef DS3231_H
#define DS3231_H

#include "main.h"
#include <stdint.h>

typedef struct
{
  uint16_t year;
  uint16_t month;
  uint16_t day;
  uint16_t weekday;
  uint16_t hour;
  uint16_t minute;
  uint16_t second;
} DS3231_Time_t;

HAL_StatusTypeDef DS3231_Init(void);
HAL_StatusTypeDef DS3231_GetTime(DS3231_Time_t *time);

#endif