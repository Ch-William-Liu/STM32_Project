#include "rtc_scheduler.h"

static uint32_t last_trigger_slot = 0;
static bool has_triggered = false;

void RTC_Scheduler_Init(void)
{
  last_trigger_slot = 0;
  has_triggered = false;
} // end function RTC_Scheduler_Init

bool RTC_Scheduler_Check(const DS3231_Time_t * time)
{
  if (time == NULL)
  {
    return false;
  } // end if time is NULL

  if ((time->minute % 5U) != 0U)
  {
    return false;
  } // end if

  uint32_t day_key = ((uint32_t)time->year * 10000UL) + ((uint32_t)time->month * 100UL) + (uint32_t)time->day;

  uint32_t slot = day_key * 288UL + (uint32_t)time->hour * 12UL + (uint32_t)time->minute / 5UL;

  if (!has_triggered || slot != last_trigger_slot)
  {
    last_trigger_slot = slot;
    has_triggered = true;
    return true;
  } // end if

  return false;
} // end function RTC_Scheduler_Check