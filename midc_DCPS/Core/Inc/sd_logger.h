#ifndef __SD_LOGGER_H
#define __SD_LOGGER_H

#include "main.h"
#include "ds3231.h"
#include "dcps5400.h"

void SD_Logger_Init(void);
void SD_WriteRawData(DS3231_Time_t *t, DCPS_data_t *data);
void SD_WriteAvgData(DCPS_Avg_t *avg);

#endif