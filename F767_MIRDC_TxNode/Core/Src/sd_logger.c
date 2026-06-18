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

    snprintf(line , sizeof(line) , "%lu,%d,%d,%d,%d,%d,%d\r\n" , timestamp , imu.acc_x , imu.acc_y , imu.acc_z , imu.gyro_x , imu.gyro_y , imu.gyro_z);

    FRESULT res = f_write(&file , line , strlen(line) , &bw);

    if (res == FR_OK)
    {
        f_sync(&file);
    } // end if ok

    return res;
} // end function SD_LogRaw