#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include <stdint.h>

typedef struct
{
    int16_t ax , ay , az;
    int16_t gx , gy , gz;
    int16_t temp_raw;
} MPU6050_Raw_t;

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit);
HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, MPU6050_Raw_t *out);
float MPU6050_TempC(int16_t temp_raw);

#endif