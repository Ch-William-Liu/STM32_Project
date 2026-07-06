#include "mpu6050.h"

#define MPU6050_ADDR        (0x68 << 1)
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_ACCEL_XOUT  0x3B

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t data[2];

    data[0] = MPU6050_PWR_MGMT_1;
    data[1] = 0x00;

    return HAL_I2C_Master_Transmit(hi2c , MPU6050_ADDR , data , 2 , 100);
} // end function MPU6050_Init 

HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c , IMU_Raw_t *imu)
{
    uint8_t reg = MPU6050_ACCEL_XOUT;
    uint8_t buf[14];

    if (HAL_I2C_Master_Transmit(hi2c , MPU6050_ADDR , &reg , 1 , 100) != HAL_OK)
    {
        return HAL_ERROR;
    } // end if I2C_Master_Transmit not OK

    if (HAL_I2C_Master_Receive(hi2c , MPU6050_ADDR , buf , 14 , 100) != HAL_OK)
    {
        return HAL_ERROR;
    } // end if I2C_Master_Transmit size not OK

    imu->acc_x  = (int16_t)(buf[0] << 8 | buf[1]);
    imu->acc_y  = (int16_t)(buf[2] << 8 | buf[3]);
    imu->acc_z  = (int16_t)(buf[4] << 8 | buf[5]);

    imu->gyro_x = (int16_t)(buf[8] << 8 | buf[9]);
    imu->gyro_y = (int16_t)(buf[10] << 8 | buf[11]);
    imu->gyro_z = (int16_t)(buf[12] << 8 | buf[13]);

    return HAL_OK;
} // end function MPU6050_ReadRaw