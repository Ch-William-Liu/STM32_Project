#ifndef PAKCET_H
#define PACKET_H

#include "stdint.h"
#include "imu_types.h"

#define PACKET_PAYLOAD_LEN  24
#define PACKET_TOTAL_LEN    38

uint16_t Packet_Build(uint8_t *packet , uint8_t freq_pair , uint8_t seq_id , uint64_t timestamp , IMU_Avg_t avg);

#endif