#ifndef __DCPS5400_H
#define __DCPS5400_H

#include "main.h"
#include "ds3231.h"
#include <stdint.h>

typedef struct
{
    float speed_cms;
    float direction_deg;
    float heading_deg;
    float pitch_deg;
    float roll_deg;
    float voltage_deg;
    uint16_t cell_index;
} DCPS_data_t;

typedef struct
{
    float avg_speed;
    float avg_direction;
    float avg_heading;
    float avg_pitch;
    float avg_roll;
    float avg_voltage;
    uint16_t sample_count;
} DCPS_Avg_t;

void DCPS_SendCommand(const char *cmd);
HAL_StatusTypeDef DCPS_ReadAndParse(DCPS_data_t *data);
void DCPS_Run_10min_Measurement(void);

#endif
