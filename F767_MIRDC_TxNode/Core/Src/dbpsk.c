#include "dbpsk_mod.h"
#include "math.h"

#define PI 3.14159265359f

float DBPSK_GetCarrierFreq(uint8_t freq_pair)
{
    switch (freq_pair)
    {
    case 1:
        return 30250.0f;
    case 2:
        return 30750.0f;
    case 3:
        return 31250.0f;
    case 4:
        return 31750.0f;
    default:
        return 30250.0f;
    } // end switch
} // end function DBPSK_GetCarriterFreq

uint32_t DBPSK_Modulate(uint8_t *packet , uint16_t packet_len , uint8_t freq_pair , uint16_t *dac_buffer , uint32_t max_samples)
{
    float fc = DBPSK_GetCarrierFreq(freq_pair);
    float phase = 0.0f;
    float phase_step = 2.0f * PI * fc / DAC_FS;

    uint32_t sample_idx = 0;

    for (uint16_t i = 0; i < packet_len; i++)
    {
        for (uint16_t b = 7; b >= 0; b--)
        {
            uint8_t bit = (packet[i] >> b) & 0x01;

            if (bit == 1)
            {
                phase += PI;

                if (phase >= 2.0f * PI)
                {
                    phase -= 2.0f * PI;
                } // end if
            } // end if

            for (uint16_t n = 0; n < SAMPLES_PER_BIT; n++)
            {
                if (sample_idx >= max_samples)
                {
                    return sample_idx;
                }

                float s = sinf(phase);

                int32_t dac_val = DAC_MID + (int32_t)(DAC_AMP * s);

                if (dac_val < 0)
                {
                    dac_val = 0;
                } // end if val < 0

                if (dac_val > 4095)
                {
                    dac_val = 4095;
                } // end if val > 4095

                dac_buffer[sample_idx] = (uint16_t)dac_val;
                sample_idx++;

                phase += phase_step;

                if (phase >= 2.0f * PI)
                {
                    phase -= 2.0f * PI;
                } // end if phase > 2Pi
            } // end for
        } // end for
    } // end for
    
    return sample_idx;
} // end function DBPSK_Modulate