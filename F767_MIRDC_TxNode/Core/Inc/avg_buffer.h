#ifndef AVG_BUFFER_H
#define AVG_BUFFER_H

#include "imu_types.h"
#include "stdint.h"

void AvgBuffer_Clear(void);
void AvgBuffer_Add(IMU_Raw_t data);
uint8_t AvgBuffer_IsFull(void);
uint8_t AvgBuffer_GetCount(void);
IMU_Avg_t AvgBuffer_GetAverage(void);

#endif