#ifndef FFT_PROCESS_H
#define FFT_PROCESS_H

#include "main.h"
#include <stdint.h>

#define FFT_MAX_SIZE 4096

void FFT_Process_Init(uint32_t fs, uint32_t fft_size);
void FFT_Process_SetConfig(uint32_t fs, uint32_t fft_size);
void FFT_Process_Frame(uint16_t *adc_buf, uint32_t len);

uint32_t FFT_Process_GetSize(void);
uint32_t FFT_Process_GetSampleRate(void);

#endif