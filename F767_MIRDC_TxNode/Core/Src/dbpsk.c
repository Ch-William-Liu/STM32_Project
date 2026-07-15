#include "dbpsk_mod.h"
#include "math.h"

#define PI                      3.14159265359f
#define CHIRP_DURATION_MS       3.5f
#define GI_DURATION_MS          0.5f

#define CHIRP_BW                200.0f  // Hz

#define CHIRP_SAMPLES           ((uint16_t)(DAC_FS * CHIRP_DURATION_MS / 1000.0f))
#define GI_SAMPLES              ((uint16_t)(DAC_FS * GI_DURATION_MS / 1000.0f))
/* switch to the chirp_version symbol */
/* 
Frequency table
    |-------------------------------------------------------------------------------------------|
    | Frequency Pair    |   Symbol  |   Center Frequency [kHz]  |   Chirp Frequency Range [kHz] |
    |-------------------|-----------|---------------------------|-------------------------------|
    |           1       |       00  |           28.2            |           28.1->28.3          |
    |           1       |       01  |           28.6            |           28.5->28.7          |
    |           1       |       10  |           29.0            |           28.9->29.1          |
    |           1       |       11  |           29.4            |           29.3->29.5          |
    |-------------------|-----------|---------------------------|-------------------------------|
    |           2       |       00  |           29.8            |           29.7->29.9          |
    |           2       |       01  |           30.2            |           30.1->30.3          |
    |           2       |       10  |           30.6            |           30.5->30.7          |
    |           2       |       11  |           31.0            |           30.9->31.1          |
    |-------------------|-----------|---------------------------|-------------------------------|
    |           3       |       00  |           31.4            |           31.3->31.5          |
    |           3       |       01  |           31.8            |           31.7->31.9          |
    |           3       |       10  |           32.2            |           32.1->32.3          |
    |           3       |       11  |           32.6            |           32.5->32.7          |
    |-------------------|-----------|---------------------------|-------------------------------|
    |           4       |       00  |           33.0            |           32.9->33.1          |
    |           4       |       01  |           33.4            |           33.3->33.5          |
    |           4       |       10  |           33.8            |           33.7->33.9          |
    |           4       |       11  |           34.2            |           34.1->34.3          |
    |-------------------|-----------|---------------------------|-------------------------------|
*/
static const float chirp_center_freq_table[4][4] = 
{
    {28200, 28600, 29000, 29400},
    {29800, 30200, 30600, 31000},
    {31400, 31800, 32200, 32600},
    {33000, 33400, 33800, 34200}
};

float FSK4_GetChirpFreq(uint8_t freq_pair, uint8_t symbol)
{
    if (freq_pair < 1 || freq_pair >4)
    {
        freq_pair = 1;
    } // end if invalid frequency pair

    if (symbol > 3)
    {
        symbol = 0;
    } // end if invalid symbol

    return chirp_center_freq_table[freq_pair - 1][symbol];
} // end function FSK4_GetChirpFreq

static uint16_t Generate_Chirp_Symbol(float center_freq, uint16_t *dac_buffer, uint32_t idx)
{
    float f_start = center_freq -  CHIRP_BW / 2.0f;
    float f_end = center_freq + CHIRP_BW / 2.0f;

    float chirp_time = (float)CHIRP_SAMPLES / DAC_FS;

    float k = (f_end - f_start) / chirp_time;

    for (uint16_t  n = 0; n < CHIRP_SAMPLES; n++)
    {
        float t = (float)n/DAC_FS;

        float phase = 2.0f * PI * (f_start * t + 0.5f * k * t * t);

        // Hann Window
        float window = 0.5f * (1.0f - cosf(2.0f * PI * (float)n / (float)(CHIRP_SAMPLES - 1)));

        float s = sinf(phase) * window;
        
        int32_t dac_val = DAC_MID + (int32_t)(DAC_AMP * s);

        if (dac_val < 0)
        {
            dac_val = 0;
        } // end if dac_val too small
        else if (dac_val > 4095) 
        {
            dac_val = 4095;
        } // end if dac_val too big

        dac_buffer[idx++] = (uint16_t)dac_val;
    } // end for n samples

    // GI
    for (uint16_t i = 0; i < GI_DURATION_MS; i++)
    {
        dac_buffer[idx++] = DAC_MID;
    } // end for GI

    return idx;
} // end function Generate_Chirp_Symbol

uint32_t FSK4_Modulate(uint8_t *packet , uint16_t packet_len , uint8_t freq_pair , uint16_t *dac_buffer , uint32_t max_samples)
{
    uint32_t sample_idx = 0;

    for (uint16_t i = 0; i < packet_len; i++)
    {
        uint8_t byte = packet[i];

        for (int b = 6; b >= 0; b-=2)
        {
            uint8_t symbol  = (byte>>b)&0x03;

            float fc = FSK4_GetChirpFreq(freq_pair , symbol);

            if (sample_idx + CHIRP_SAMPLES + GI_SAMPLES >= max_samples)
            {
                return sample_idx;
            } // end if idx too much

            sample_idx = Generate_Chirp_Symbol(fc , dac_buffer , sample_idx);
        } // end for each byte
    } // end for i in packet

    return sample_idx;
} // end function FSK4_Modulate