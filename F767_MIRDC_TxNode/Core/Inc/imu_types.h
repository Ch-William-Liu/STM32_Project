#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdint.h>

typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

} IMU_Raw_t;


typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;
} IMU_Avg_t;

#endif