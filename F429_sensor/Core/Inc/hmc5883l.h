#ifndef HMC5883L_H
#define HMC5883L_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include <stdint.h>

typedef struct
{
    int16_t mx , my , mz;
} HMC5883L_Raw_t;

HAL_StatusTypeDef HMC5883L_Init(I2C_HandleTypeDef *hi2c , uint8_t addr_7bit);
HAL_StatusTypeDef HMC5883L_ReadRaw(I2C_HandleTypeDef *hi2c , uint8_t addr_7bit , HMC5883L_Raw_t *out);

#endif