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

FRESULT SD_LogRaw(uint64_t timestamp , IMU_Raw_t imu)
{
    char line[128];

    uint32_t timestamp_high = timestamp / 1000000000ULL;
    uint32_t timestamp_low  = timestamp % 1000000000ULL;


    snprintf(line , sizeof(line),
             "%lu%09lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
             timestamp_high, timestamp_low,
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

FRESULT SD_LogTxPacket(uint64_t timestamp, uint16_t seq_id, uint8_t freq_pair, const uint8_t *packet, uint16_t packet_len)
{
    FIL tx_file;
    FRESULT res;
    UINT bw;

    char line[256];
    char hex_str[129];

    uint32_t timestamp_high , timestamp_low;

    if (packet == NULL || packet_len == 0)
    {
        printf("TXLOG invalid packet\r\n");
        return FR_INVALID_PARAMETER;
    } // end if packet is null

    if (packet_len > 64)
    {
        printf("TXLOG packet too long: %u\r\n" , (unsigned int)packet_len);
        return FR_INVALID_PARAMETER;
    } // end if packet too long

    timestamp_high = (uint32_t)(timestamp / 1000000000ULL);
    timestamp_low = (uint32_t)(timestamp % 1000000000ULL);

    for (uint16_t i = 0; i < packet_len; i++)
    {
        int written = snprintf(&hex_str[i * 2], sizeof(hex_str) - (i * 2), "%02X" , (unsigned int)packet[i]);

        if (written != 2)
        {
            printf("TXLOG HEX conversion failed at %u\r\n", (unsigned int)i);
            return FR_INVALID_PARAMETER;
        } // end if written
    } // end for

    hex_str[packet_len * 2] = '\0';

    res = f_open(&tx_file , "TX_LOG.csv" , FA_OPEN_APPEND | FA_WRITE);

    printf("TXLOG f_open res=%d\r\n" , res);

    if (res != FR_OK)
    {
        return res;
    } // end if open file ok

    if (f_size(&tx_file) == 0)
    {
        static const char header[] = "Timestamp,SeqID,FreqPair,PacketLen,PacketHex\r\n";

        res = f_write(&tx_file , header , sizeof(header) - 1 , &bw);

        printf("TXLOG header: res=%d, bw=%u\r\n" , res , bw);

        if (res != FR_OK || bw != (UINT)(sizeof(header) - 1))
        {
            f_close(&tx_file);
            return (res != FR_OK) ? res : FR_DISK_ERR;
        } // end if res no OK or bw is invalid
    } // end if empty file

    int line_len = snprintf(line, sizeof(line) , "%lu%09lu,%u,%u,%u,%s\r\n" , 
        (unsigned long)timestamp_high , (unsigned long)timestamp_low, (unsigned int)seq_id , (unsigned int)freq_pair , (unsigned int)packet_len , hex_str);

    if (line_len <= 0 || line_len >= (int)sizeof(line))
    {
        printf("TXLOG line formatting failed: %d\r\n" , line_len);

        f_close(&tx_file);
        return FR_INVALID_PARAMETER;
    } // end if line_len too big

    printf("TXLOG line: %s", line);

    res = f_write(&tx_file , line , (UINT)line_len , &bw);

    printf("TXLOG f_write res=%d, bw=%u, len=%d\r\n" , res, bw, line_len);

    if (res == FR_OK && bw == (UINT)line_len)
    {
        res = f_sync(&tx_file);
        printf("TXLOG f_sync res=%d\r\n" , res);
    } // end of res is OK
    else if (res == FR_OK)
    {
        res = FR_DISK_ERR;
        printf("TXLOG incomplete write\r\n");
    } // end if 

    FRESULT close_res = f_close(&tx_file);

    if (res == FR_OK && close_res != FR_OK)
    {
        res = close_res;
    } // end if

    return res;
} // end function SD_LogTxPacket
