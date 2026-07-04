#include "dbpsk_mod.h"
#include "math.h"

#define PI 3.14159265359f

static const float fsk4_table[4][4] = 
{
    {28200.0f, 28600.0f, 29000.0f, 29400.0f},
    {29700.0f, 30100.0f, 30500.0f, 30900.0f},
    {31200.0f, 31600.0f, 32000.0f, 32400.0f},
    {32700.0f, 33100.0f, 33500.0f, 33900.0f}
};

float FSK4_GetToneFreq(uint8_t freq_pair , uint8_t symbol)
{
if (freq_pair < 1 || freq_pair > 4)
    {
        freq_pair = 1;
    } // end if unvalid freq_pair
    
    if (symbol > 3)
    {
        symbol = 0;
    } // end if unvalid symbol

    return fsk4_table[freq_pair - 1][symbol];
} // end function FSK4_GetToneFreq

uint32_t FSK4_Modulate(uint8_t *packet , uint16_t packet_len , uint8_t freq_pair , uint16_t *dac_buffer , uint32_t max_samples)
{
    uint32_t sample_idx = 0;
    float phase = 0.0f;


    for (uint16_t i = 0; i < packet_len; i++)
    {
        uint8_t byte = packet[i];

        for (int b = 6; b >= 0; b -= 2)
        {
            uint8_t symbol = (byte >> b) & 0x03;

            float f = FSK4_GetToneFreq(freq_pair , symbol);
            float phase_step = 2.0f * PI * f / DAC_FS;

            for (uint16_t n = 0; n < SAMPLES_PER_SYMBOL; n++)
            {
                if (sample_idx >= max_samples)
                {
                    return sample_idx;
                } // end if reach max samples

                float s = sinf(phase);

                int32_t dac_val = DAC_MID + (int32_t)(DAC_AMP * s);

                if (dac_val < 0)
                {
                    dac_val = 0;
                } // end if dac_val smaller than 0 (due to the build-in dac output range: 0~4095)

                if (dac_val > 4095)
                {
                    dac_val = 4095;
                } // end if dac_val greater than 4095

                dac_buffer[sample_idx++] = (uint16_t)dac_val;

                phase += phase_step;

                if (phase > 2.0f * PI)
                {
                    phase -= 2.0f * PI;
                } // end if phase
            } // end for sample per symbol
        } // end for byte
    } // end for packet len 

    return sample_idx;
} // end function FSK4_Modulate