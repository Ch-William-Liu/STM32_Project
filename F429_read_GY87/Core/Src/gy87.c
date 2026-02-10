#include "gy87.h"

HAL_StatusTypeDef GY87_Init(I2C_HandleTypeDef *hi2c , GY87_Addr_t addr)
{
    if (MPU6050_Init(hi2c , addr.mpu_addr_7bit) != HAL_OK) return HAL_ERROR;
    if (HMC5883L_Init(hi2c , addr.mag_addr_7bit) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
} // end GY87_Init

HAL_StatusTypeDef GY87_Read(I2C_HandleTypeDef *hi2c , GY87_Addr_t addr , GY87_Data_t *out)
{
    if (MPU6050_ReadRaw(hi2c , addr.mpu_addr_7bit , &out->mpu) != HAL_OK) return HAL_ERROR;
    if (HMC5883L_ReadRaw(hi2c , addr.mag_addr_7bit , &out->mag) != HAL_OK) return HAL_ERROR;

    out->tempC = MPU6050_TempC(out->mpu.temp_raw);
    return HAL_OK;
} // end GY87_Read