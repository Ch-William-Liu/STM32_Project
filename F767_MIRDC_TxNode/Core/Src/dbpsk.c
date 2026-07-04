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

    } // end if unvalid freq_pair
} // end function FSK4_GetToneFreq

uint32_t FSK4_Moudlate(uint8_t *packet , uint16_t packer_len , uint8_t freq_pair , uint16_t *dac_buffer , uint32_t max_samples)
{

} // end function FSK4_Modulate