#ifndef SD_SPI_H
#define SD_SPI_H

#include "main.h"
#include <stdint.h>

#define SD_BLOCK_SIZE 512

uint8_t SD_SPI_Init(void);
uint8_t SD_SPI_ReadBlocks(uint8_t *buf, uint32_t sector, uint32_t count);
uint8_t SD_SPI_WriteBlocks(const uint8_t *buf, uint32_t sector, uint32_t count);

#endif