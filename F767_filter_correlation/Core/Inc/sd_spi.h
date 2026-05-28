#ifndef SD_SPI_H
#define SD_SPI_H

#include "main.h"
#include "fatfs.h"
#include "spi.h"

DSTATUS SD_SPI_initialize(BYTE pdrv);
DSTATUS SD_SPI_status(BYTE pdrv);
DRESULT SD_SPI_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
DRESULT SD_SPI_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
DRESULT SD_SPI_ioctl(BYTE pdrv, BYTE cmd, void *buff);

#endif