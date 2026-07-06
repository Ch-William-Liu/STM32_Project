#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include "main.h"
#include "imu_types.h"
#include "stdint.h"
#include "fatfs.h"

FRESULT SD_Logger_Init(void);
FRESULT SD_LogRaw(uint32_t timestamp , IMU_Raw_t imu);
FRESULT SD_LogTxPacket(uint32_t timestamp, uint16_t seq_id, uint8_t freq_pair, uint8_t *packet, uint16_t packet_len);

#endif