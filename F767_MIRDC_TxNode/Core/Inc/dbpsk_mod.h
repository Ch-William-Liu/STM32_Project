#ifndef DBPSK_MODE_H
#define DBPSK_MODE_H

#include "stdint.h"

#define DAC_FS              192000.0f
#define BIT_RATE            500.0f
#define FSK_M               4
#define BITS_PER_SYMBOL     2
#define SYMBOL_RATE         250.0f
#define SYMBOL_DURATION_MS  4.0f

#define SAMPLES_PER_SYMBOL  768     // 192000 * 0.004
#define DAC_MID             2048
#define DAC_AMP             1200

float FSK4_GetToneFreq(uint8_t freq_pair , uint8_t symbol);
uint32_t FSK4_Modulate(uint8_t *packet , uint16_t packer_len , uint8_t freq_pair , uint16_t *dac_buffer , uint32_t max_samples);

#endif