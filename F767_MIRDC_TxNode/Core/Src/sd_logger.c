#include "sd_logger.h"
#include "fatfs.h"
#include "stdio.h"
#include "string.h"

static FATFS fs;
static FIL file;
static UINT bw;

FRESULT SD_Logger_Init(void)
{
    FRESULT res;

    res = f_mount(&fs , "" , 1);
    printf("f_mount res = %d\r\n", res);
    if (res != FR_OK)
    {
        return res;
    } // if res not OK

    res = f_open(&file , "IMU_RAW.csv" , FA_OPEN_APPEND | FA_WRITE);
    if (res != FR_OK)
    {
        return res;
    } // if res not open successfully

    if (f_size(&file) == 0)
    {
        char header[] = "Timestamp,AccX,AccY,AccZ,GyroX,GyroY,GyroZ\r\n";
        f_write(&file , header , strlen(header) , &bw);
        f_sync(&file);
    } // end if file_size == 0

    return FR_OK;
} // end function SD_Logger_Init

FRESULT SD_LogRaw(uint32_t timestamp , IMU_Raw_t imu)
{
    char line[128];

    snprintf(line , sizeof(line),
             "%lu,%d,%d,%d,%d,%d,%d\r\n",
             timestamp,
             imu.acc_x, imu.acc_y, imu.acc_z,
             imu.gyro_x, imu.gyro_y, imu.gyro_z);

    FRESULT res = f_write(&file , line , strlen(line) , &bw);

    printf("f_write res=%d, bw=%u, len=%u\r\n",
           res, bw, (unsigned int)strlen(line));

    if (res == FR_OK && bw == strlen(line))
    {
        res = f_sync(&file);
        printf("f_sync res=%d\r\n", res);
    }
    else
    {
        printf("RAW write failed\r\n");
    }

    return res;
} // end function SD_LogRaw

FRESULT SD_LogTxPacket(uint32_t timestamp, uint16_t seq_id, uint8_t freq_pair, uint8_t *packet, uint16_t packet_len)
{
    FIL tx_file;
    FRESULT res;
    UINT bw;
    char line[1024];
    char hex_str[512];

    hex_str[0] = '\0';

    for (uint16_t i = 0; i < packet_len; i++)
    {
        char temp[4];
        snprintf(temp, sizeof(temp), "%02X", packet[i]);
        strncat(hex_str, temp, sizeof(hex_str) - strlen(hex_str) - 1);
    } // end for

    res = f_open(&tx_file, "TX_PACKET_LOG.csv", FA_OPEN_APPEND | FA_WRITE);
    if (res != FR_OK)
    {
        return res;
    } // end if write file not ok

    if (f_size(&tx_file) == 0)
    {
        char header[] = "Timestamp,SeqID,FreqPair,PacketLen,PacketHex\r\n";
        f_write(&tx_file, header, strlen(header), &bw);
    } // end if empty file

    snprintf(line, sizeof(line), "%lu,%u,%u,%u,%s" , timestamp, seq_id, freq_pair, packet_len, hex_str);

    res = f_write(&tx_file, line, strlen(line), &bw);
    
    if (res == FR_OK)
    {
        res = f_sync(&tx_file);
    } // end if write ok

    f_close(&tx_file);

    return res;
} // end function SD_LogTxPacket