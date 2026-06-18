#include "crc16.h"

uint16_t CRC16_CCITT(uint8_t *data , uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= ((uint16_t)data[i] << 8);

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000) 
            {
                crc = (crc << 1) ^ 0x1021;
            } // end crc
            else
            {
                crc << 1;
            } // end else
        } // end for
    } // end for

    return crc;
} // end function CRC16_CCITT