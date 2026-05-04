#ifndef WAV_RECORDER_H
#define WAV_RECORDER_H

#include "main.h"
#include <stdint.h>

void WAV_Recorder_Init(void);
void WAV_Recorder_Process(void);

uint8_t WAV_Recorder_Start(uint32_t duration_sec, uint32_t sample_rate);
void WAV_Recorder_Stop(void);

void WAV_Recorder_WriteADC(uint16_t *adc_buf, uint32_t len);

uint8_t WAV_Recorder_IsRecording(void);

#endif