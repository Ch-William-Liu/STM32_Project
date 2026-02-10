#include "mpu6050.h"
#include <stdint.h>

#define REG_PWR_MGMT_1      0x6B
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B

static inline uint16_t i2c_addr8(uint8_t addr_7bit)
{
    return (uint16_t)(addr_7bit << 1);
} // end i2c_addr8

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit)
{
    uint8_t data = 0x00; // wake up
    if (HAL_I2C_Mem_Write(hi2c , i2c_addr8(addr_7bit) , REG_PWR_MGMT_1 , 1 , &data , 1 , 100) != HAL_OK)
    {
        return HAL_ERROR;
    } // end if 
    
    data = 0x07; // sample rate divider
    HAL_I2C_Mem_Write(hi2c , i2c_addr8(addr_7bit) , REG_SMPLRT_DIV , 1 , &data , 1 ,100);

    data = 0x06; // DLPF config
    HAL_I2C_Mem_Write(hi2c , i2c_addr8(addr_7bit) , REG_CONFIG , 1 , &data , 1 , 100);

    data = 0x00; // gyro ±250 dps
    HAL_I2C_Mem_Write(hi2c , i2c_addr8(addr_7bit) , REG_GYRO_CONFIG , 1 , &data , 1 , 100);

    data = 0x00; // accel ±2g
    HAL_I2C_Mem_Write(hi2c , i2c_addr8(addr_7bit) , REG_ACCEL_CONFIG , 1, &data , 1 , 100);

    return HAL_OK;
} // end MPU6050_Init

HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c , uint8_t addr_7bit , MPU6050_Raw_t *out)
{
    uint8_t buf[14];

    if (HAL_I2C_Mem_Read(hi2c , i2c_addr8(addr_7bit) , REG_ACCEL_XOUT_H , 1 , buf , 14 , 200) != HAL_OK)
    {
        return HAL_ERROR;
    } // end if 

    out->ax = (int16_t)((buf[0] << 8) | buf[1]);
    out->ay = (int16_t)((buf[2] << 8) | buf[3]);
    out->ax = (int16_t)((buf[4] << 8) | buf[5]);
    out->temp_raw = (int16_t)((buf[6] << 8) | buf[7]);
    out->gx = (int16_t)((buf[8] << 8) | buf[9]);
    out->gy = (int16_t)((buf[10] << 8) | buf[11]);
    out->gz = (int16_t)((buf[12] << 8) | buf[13]);

    return HAL_OK;
} // end MPU6050_ReadRaw

float MPU6050_TempC(int16_t temp_raw)
{
    // Datasheet: TempC = (Temp_out / 340) + 36.53
    return ((float)temp_raw / 340.0f) + 36.53f;
}