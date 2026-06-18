#include "dcps5400.h"
#include "sd_logger.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

extern UART_HandleTypeDef huart2;       // DCPS
extern UART_HandleTypeDef huart3;       // PC

#define DCPS_UART huart2
#define PC_UART huart3

void DCPS_SendCommand(const char *cmd)
{
    char tx[64];
    snprintf(tx , sizeof(tx) , "%s\r\n" , cmd);
    HAL_UART_Transmit(&DCPS_UART , (uint8_t *)tx , strlen(tx) , 1000);
} // end function DCPS_SendCommand

HAL_StatusTypeDef DCPS_ReadLine(char *line , uint16_t max_len , uint32_t timeout)
{
    uint8_t ch;
    uint16_t idx = 0;
    uint32_t tickstart = HAL_GetTick();

    while ((HAL_GetTick() - tickstart) < timeout)
    {
        if (HAL_UART_Receive(&DCPS_UART , &ch , 1 , 20) == HAL_OK)
        {
            if (ch == '\n')
            {
                line[idx] = '\0';
                return HAL_OK;
            } // end if

            if (ch != '\r' && idx < max_len - 1)
            {
                line[idx++] = ch;
            } // end if
        } // end if
    } // end while
    
    line[idx] = '\0';
    return HAL_TIMEOUT;
} // end function DCPS_ReadLine 

HAL_StatusTypeDef DCPS_ReadAndParse(DCPS_data_t *data)
{
    char line[256];

    while (DCPS_ReadLine(line , sizeof(line) , 3000) == HAL_OK)
    {
        if (strstr(line , "Horizontal Speed") != NULL)
        {
            sscanf(line , "%f" , &data->speed_cms);
        } // end if Horizontal Speed
        else if (strstr(line , "Direction") != NULL)
        {
            sscanf(line , "%f" , &data->direction_deg);
        } // end if Direction
        else if (strstr(line , "Heading") != NULL)
        {
            sscanf(line , "%f" , &data->heading_deg);
        } // end if Heading
        else if (strstr(line , "Pitch") != NULL)
        {
            sscanf(line , "%f" , &data->pitch_deg);
        } // end if Pitch
        else if (strstr(line , "Roll") != NULL)
        {
            sscanf(line , "%f" , &data->roll_deg);
        } // end if Roll
        else if (strstr(line , "Voltage") != NULL)
        {
            sscanf(line , "%f" , &data->voltage_deg);
            return HAL_OK;
        } // end if voltage
    } // end while
    
    return HAL_ERROR;
} // end function DCPS_ReadAndParse

static void PC_SendAvgData(DCPS_Avg_t *avg)
{
    char msg[256];
    snprintf(msg , sizeof(msg) , "AVG,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u\r\n",
    avg->avg_speed , avg->avg_direction , avg->avg_heading , avg->avg_pitch , avg->avg_roll , avg->avg_voltage , avg->sample_count);

    HAL_UART_Transmit(&PC_UART , (uint8_t *)msg , strlen(msg) , 1000);
} // end function PC_SendAvgData

void DCPS_Run_10min_Measurement(void)
{
    DCPS_data_t data;
    DCPS_Avg_t avg = {0};

    float dir_x = 0.0f;
    float dir_y = 0.0f;
    uint16_t count = 0;

    DCPS_SendCommand("Start");
    HAL_Delay(1000);

    for (uint16_t i = 0; i < 60; i++)
    {
        DS3231_Time_t now;
        DS3231_GetTime(&now);

        DCPS_SendCommand("Do Sample");
        HAL_Delay(1000);
        DCPS_SendCommand("Do Output");

        if (DCPS_ReadAndParse(&data) == HAL_OK)
        {
            SD_WriteRawData(&now , &data);

            avg.avg_speed += data.speed_cms;
            avg.avg_heading += data.heading_deg;
            avg.avg_pitch += data.pitch_deg;
            avg.avg_roll += data.roll_deg;
            avg.avg_voltage += data.voltage_deg;

            float rad = data.direction_deg * 3.1415926f / 180.0f;
            dir_x += cosf(rad);
            dir_y += sinf(rad);

            count++;
        } // end if
        HAL_Delay(10000);
    } // end for

    DCPS_SendCommand("Stop");

    if (count > 0)
    {
        avg.sample_count = count;
        avg.avg_speed /= count;
        avg.avg_heading /= count;
        avg.avg_pitch /= count;
        avg.avg_roll /= count;
        avg.avg_voltage /= count;

        avg.avg_direction = atan2(dir_y , dir_x) * 180.0f / 3.1415926f;

        if (avg.avg_direction < 0)
        {
            avg.avg_direction += 360.0f;
        } // end if

        SD_WriteAvgData(&avg);
        PC_SendAvgData(&avg);
    } // end if
} // DCPS_Run_10min_Measurement