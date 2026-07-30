#ifndef CHIRP_OUTPUT_H
#define CHIRP_OUTPUT_H

#include "main.h"
#include <stdint.h>

/* TIM6 update frequency must equal this value. */
#define CHIRP_DAC_FS_HZ             96000U
#define CHIRP_START_FREQ_HZ         3000.0f
#define CHIRP_END_FREQ_HZ           7000.0f
#define CHIRP_DURATION_SEC          2.0f

#define CHIRP_DAC_MID               2048U
#define CHIRP_DAC_AMP               1200.0f

/* 1024 samples, 512 samples per DMA half.
 * At 96 kHz, each half is 5.333 ms and 192000 samples is exactly
 * divisible by 512, allowing the stream to stop at exactly 2 s.
 */
#define CHIRP_STREAM_BUFFER_SIZE    1024U
#define CHIRP_STREAM_HALF_SIZE      (CHIRP_STREAM_BUFFER_SIZE / 2U)

void ChirpPlayer_Init(DAC_HandleTypeDef *dac_handle,
                      TIM_HandleTypeDef *tim_handle);

HAL_StatusTypeDef ChirpPlayer_Play(void);
void ChirpPlayer_Process(void);
void ChirpPlayer_Stop(void);

uint8_t ChirpPlayer_IsRunning(void);
uint8_t ChirpPlayer_IsFinished(void);
uint8_t ChirpPlayer_HasError(void);

#endif /* CHIRP_OUTPUT_H */
