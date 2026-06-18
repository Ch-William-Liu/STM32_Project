#include "sd_logger.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>

static FATFS fs;
static FIL file;

void SD_Logger_Init(void)
{
    if (f_mount(&fs , "" , 1) == FR_OK)
    {
        f_open(&file , "DCPS_RAW.csv" , FA_OPEN_APPEND | FA_WRITE);
        if (f_size(&file) == 0)
        {
            f_puts("datetime,speed_cms,direction_deg,heading_deg,pitch_deg,roll_deg,voltage_v,cell_index\r\n",&file);
        } // end if empty file
        fclose(&file);

        f_open(&file , "DCPS_AVG.csv" , FA_OPEN_APPEND | FA_WRITE);
        if (f_size(&file) == 0)
        {
            f_puts("avg_speed_cms,avg_direction,avg_heading,avg_pitch_deg,avg_roll_deg,avg_voltage_v,sample_count\r\n",&file);
        } // end if
        f_close(&file);
    } // end if
} // end function SD_Logger_Init

void SD_WriteRawData(DS3231_Time_t *t, DCPS_data_t *data)
{
    char line[256];
    snprintf(line , sizeof(line) , "20%02u-%02u-%02u %02u:%02u:%02u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u\r\n",
    t->year , t->month , t->date , t->day , t->hour , t->min , t->sec ,
    data->speed_cms , data->direction_deg , data->heading_deg , data->pitch_deg , data->roll_deg , data->voltage_deg , data->cell_index);

    if (f_open(&file , "DCPS_RAW.csv" , FA_OPEN_APPEND | FA_WRITE) == FR_OK)
    {
        f_puts(line , &file);
        f_close(&file);
    } // end if
} // end function SD_WriteRawData

void SD_WriteAvgData(DCPS_Avg_t *avg)
{
    char line[256];

    snprintf(line , sizeof(line) , "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u\r\n" , 
    avg->avg_speed , avg->avg_direction , avg->avg_heading , avg->avg_heading , avg->avg_pitch , avg->avg_roll , avg->avg_voltage , avg->sample_count);

    if (f_open(&file , "DCPS_AVG.csv" , FA_OPEN_APPEND | FA_WRITE) == FR_OK)
    {
        f_puts(line , &file);
        f_close(&file);
    } // end if 
} // end function SD_WriteAvgData