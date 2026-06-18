#ifndef DBPSK_MODE_H
#define DBPSK_MODE_H

#include "stdint.h"

#define DAC_FS          192000.0f
#define BIT_RATE        500.0f
#define SAMPLES_PER_BIT 384
#define DAC_MID         2048
#define DAC_AMP         1200

float DBPSK_GetCarrierFreq(uint8_t freq_pair);
uint32_t DBPSK_Modulate(uint8_t *packet , uint16_t packet_len , uint8_t freq_pair , uint16_t *dac_buffer , uint32_t max_samples);


#endif