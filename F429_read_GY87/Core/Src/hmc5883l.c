#include "hmc5883l.h"
#include <stdint.h>

#define REG_CONFIG_A    0x00
#define REG_CONFIG_B    0x01
#define REG_MODE        0x02
#define REG_DATA_X_MSB  0x03

static inline uint16_t i2c_addr8(uint8_t addr_7bit)
{
    return (uint16_t)(addr_7bit << 1);
} // end i2c_addr8

HAL_StatusTypeDef HMC5883L_Init(I2C_HandleTypeDef *hi2c , uint8_t addr_7bit)
{
    // Config A: 8-average, 15 Hz, normal measurement
    uint8_t cfgA = 0x70;
    if (HAL_I2C_Mem_Write(hi2c , i2c_addr8(addr_7bit) , REG_CONFIG_A , 1 , &cfgA , 1 , 100) != HAL_OK)
    {
        return HAL_ERROR;
    } // end if

    // Config B: Gain
    uint8_t cfgB = 0xA0;
    HAL_I2C_Mem_Write(hi2c , i2c_addr8(addr_7bit) , REG_CONFIG_B , 1 , &cfgB , 1 , 100);

    // Mode: continuous measurement
    uint8_t mode = 0x00;
    HAL_I2C_Mem_Write(hi2c , i2c_addr8(addr_7bit) , REG_MODE , 1 , &mode , 1 , 100);

    return HAL_OK;
} // end HMC5883L_Init

HAL_StatusTypeDef HMC5883L_ReadRaw(I2C_HandleTypeDef *hi2c , uint8_t addr_7bit , HMC5883L_Raw_t *out)
{
    uint8_t buf[6];

    if (HAL_I2C_Mem_Write(hi2c , i2c_addr8(addr_7bit) , REG_DATA_X_MSB , 1 , buf , 6 , 200) != HAL_OK)
    {
        return HAL_ERROR;
    } // end if 

    // HMC5883L output order: X , Z , Y
    int16_t x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t z = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t y = (int16_t)((buf[4] << 8) | buf[5]);

    out->mx = x;
    out->my = y;
    out->mz = z;

    return HAL_OK;
} // end HMC5883L_ReadRaw