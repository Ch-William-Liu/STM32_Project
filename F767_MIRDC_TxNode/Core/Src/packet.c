#include "packet.h"
#include "crc16.h"
#include "string.h"

uint16_t Packet_Build(uint8_t *packet , uint8_t freq_pair , uint8_t seq_id , uint8_t timestamp , IMU_Avg_t avg)
{
    uint16_t idx = 0;

    // Preamble
    packet[idx++] = 0xAA;
    packet[idx++] = 0xAA;

    //Sync word
    packet[idx++] = 0xA5;
    packet[idx++] = 0x5A;

    // Frequency Pair
    packet[idx++] = freq_pair;

    // Sequence ID
    packet[idx++] = (seq_id >> 8) & 0xFF;
    packet[idx++] = seq_id & 0xFF;

    // Timestamp
    packet[idx++] = (timestamp >> 24) & 0xFF;
    packet[idx++] = (timestamp >> 16) & 0xFF;
    packet[idx++] = (timestamp >> 8) & 0xFF;
    packet[idx++] = timestamp & 0xFF;

    // Payload length
    packet[idx++] = PACKET_PAYLOAD_LEN;

    // ACC X/Y/Z
    memcpy(&packet[idx] , &avg.acc_x , 4);
    idx += 4;

    memcpy(&packet[idx] , &avg.acc_y , 4);
    idx += 4;

    memcpy(&packet[idx] , &avg.acc_z , 4);
    idx += 4;

    // Gyro X/Y/Z
    memcpy(&packet[idx] , &avg.gyro_x , 4);
    idx += 4;

    memcpy(&packet[idx] , &avg.gyro_y , 4);
    idx += 4;

    memcpy(&packet[idx] , &avg.gyro_z , 4);
    idx += 4;

    // CRC-16
    uint16_t crc = CRC16_CCITT(packet , idx);

    packet[idx++] = (crc >> 8) & 0xFF;
    packet[idx++] = crc & 0xFF;

    return idx;
} // end function Packet_Build