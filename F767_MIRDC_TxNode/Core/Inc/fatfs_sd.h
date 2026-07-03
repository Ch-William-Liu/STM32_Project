#ifndef __FATFS_SD_H
#define __FATFS_SD_H

#include "stm32f7xx_hal.h"
#include "main.h"
#include "stdint.h"
extern SPI_HandleTypeDef hspi1;

#define SD_SPI_HANDLE hspi1
uint8_t SD_SPI_Init(void);
uint8_t SD_SPI_ReadBlocks(uint8_t *buff, uint32_t sector, uint32_t count);
uint8_t SD_SPI_WriteBlocks(const uint8_t *buff, uint32_t sector, uint32_t count);
uint8_t SD_SPI_GetSectorCount(uint32_t *sector_count);

#endif