#ifndef RTC_SCHEDULER_H
#define RTC_SCHEDULER_H

#include "ds3231.h"
#include <stdbool.h>

void RTC_Scheduler_Init(void);
bool RTC_Scheduler_Check(const DS3231_Time_t *time);

#endif