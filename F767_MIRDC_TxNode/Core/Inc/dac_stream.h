#ifndef DAC_STREAM_H
#define DAC_STREAM_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

#define DAC_STREAM_BUFFER_SIZE        4096U
#define DAC_STREAM_HALF_SIZE          (DAC_STREAM_BUFFER_SIZE / 2U)

#define DAC_STREAM_PAIR_COUNT         4U
#define DAC_STREAM_MAX_PACKET_SIZE    64U

HAL_StatusTypeDef DAC_Stream_StartSequence(const uint8_t packets[DAC_STREAM_PAIR_COUNT][DAC_STREAM_MAX_PACKET_SIZE], const uint16_t packet_lengths[DAC_STREAM_PAIR_COUNT]);
void DAC_Stream_Init(void);
void DAC_Stream_Process(void);

void DAC_Stream_Stop(void);

uint8_t DAC_Stream_IsRunning(void);
uint8_t DAC_Stream_IsFinished(void);
uint8_t DAC_Stream_HasError(void);

#endif