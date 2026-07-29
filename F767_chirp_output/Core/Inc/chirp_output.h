#ifndef CHIRP_OUTPUT_H
#define CHIRP_OUTPUT_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

#define DAC_FS              100000U
#define DAC_MID             2048U
#define DAC_AMP             1200U

#define CHIRP_START_FREQ    3000.0f
#define CHIRP_END_FREQ      7000.0f
#define CHIRP_DURATION_SEC  2.0f

#define DAC_STREAM_BUFFER_SIZE    4096U
#define DAC_STREAM_HALF_SIZE      (DAC_STREAM_BUFFER_SIZE / 2U)

void ChirpPlayer_Init(DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim);
HAL_StatusTypeDef ChirpPlayer_Play(void);
void ChirpPlayer_Stop(void);
uint8_t ChirpPlayer_IsRunning(void);
uint8_t ChirpPlayer_HasError(void);

#endif