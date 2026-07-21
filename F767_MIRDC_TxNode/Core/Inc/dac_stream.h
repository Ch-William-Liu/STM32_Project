#ifndef DAC_STREAM_H
#define DAC_STREAM_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

#define DAC_STREAM_BUFFER_SIZE        4096U
#define DAC_STREAM_HALF_SIZE          (DAC_STREAM_BUFFER_SIZE / 2U)

HAL_StatusTypeDef DAC_Stream_Start(void);             // Start stream
void DAC_Stream_Stop(void);                           // stop dma and back to mid-point
uint8_t DAC_Stream_IsRunning(void);                   // return is running or not

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac);

#endif