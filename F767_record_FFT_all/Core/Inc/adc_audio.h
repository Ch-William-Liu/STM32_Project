#ifndef ADC_AUDIO_H
#define ADC_AUDIO_H

#include "main.h"
#include <stdint.h>

#define ADC_DMA_SIZE 8192

extern uint16_t adc_dma_buf[ADC_DMA_SIZE];

void ADC_Audio_Init(void);
void ADC_Audio_Start(void);
void ADC_Audio_Stop(void);

void ADC_Audio_ProcessHalf(void);
void ADC_Audio_ProcessFull(void);

void ADC_Audio_SetSampleRate(uint32_t fs);
uint32_t ADC_Audio_GetSampleRate(void);

#endif