#ifndef GY87_H
#define GY87_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include <stdint.h>

#include "mpu6050.h"
#include "hmc5883l.h"

typedef struct 
{
    uint8_t mpu_addr_7bit; // usually 0x68 or 0x69
    uint8_t mag_addr_7bit; // usually 0x1E
} GY87_Addr_t;

typedef struct 
{
    MPU6050_Raw_t mpu;
    HMC5883L_Raw_t mag;
    float tempC;
} GY87_Data_t;

HAL_StatusTypeDef GY87_Init(I2C_HandleTypeDef *hi2c , GY87_Addr_t addr);
HAL_StatusTypeDef GY87_Read(I2C_HandleTypeDef *hi2c , GY87_Addr_t addr , GY87_Data_t *out);

#endif