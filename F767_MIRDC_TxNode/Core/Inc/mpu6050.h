#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include "imu_types.h"

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c , IMU_Raw_t *imu);

#endif