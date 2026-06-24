#include "avg_buffer.h"

#define AVG_COUNT 70

static IMU_Raw_t imu_buffer[AVG_COUNT];
static uint8_t imu_count = 0;

void AvgBuffer_Clear(void)
{
    imu_count = 0;
} // end function AvgBuffer_Clear

void AvgBuffer_Add(IMU_Raw_t data)
{
    if (imu_count < AVG_COUNT)
    {
        imu_buffer[imu_count] = data;
        imu_count++;
    } // end if count length less than buffer length
} // end function AvgBuffer_Add

uint8_t AvgBuffer_IsFull(void)
{
    return (imu_count >= AVG_COUNT);
} // end function AvgBuffer_IsFull

uint8_t AvgBuffer_GetCount(void)
{
    return imu_count;
} // end function AvgBuffer_GetConut

IMU_Avg_t AvgBuffer_GetAverage(void)
{
    IMU_Avg_t avg = {0};

    if (imu_count == 0)
    {
        return avg;
    } // end if count == 0

    for (uint8_t i = 0; i < imu_count; i++)
    {
        avg.acc_x   += imu_buffer[i].acc_x;
        avg.acc_y   += imu_buffer[i].acc_y;
        avg.acc_z   += imu_buffer[i].acc_z;

        avg.gyro_x  += imu_buffer[i].gyro_x;
        avg.gyro_y  += imu_buffer[i].gyro_y;
        avg.gyro_z  += imu_buffer[i].gyro_z;
    } // end for

    avg.acc_x   /= imu_count;
    avg.acc_y   /= imu_count;
    avg.acc_z   /= imu_count;

    avg.gyro_x  /= imu_count;
    avg.gyro_y  /= imu_count;
    avg.gyro_z  /= imu_count;

    return avg;
} // end function AvgBuffer_GetAverage