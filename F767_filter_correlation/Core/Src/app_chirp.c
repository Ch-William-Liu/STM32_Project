#include "app_chirp.h"
#include "app_config.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

void APP_Chirp_Generate(float *chirpTemplate, uint32_t length)
{
    float T = (float)length / (float)APP_FS_HZ;
    float k = (APP_CHIRP_F1_HZ - APP_CHIRP_F0_HZ) / T;

    for (uint32_t n = 0; n < length; n++)
    {
        float t = (float)n / (float)APP_FS_HZ;

        float phase = 2.0f * PI * (APP_CHIRP_F0_HZ * t + 0.5f * k * t * t);
        float chirpValue = sinf(phase);

        float window = 0.5f - 0.5f * cosf((2.0f * PI * (float)n) / ((float)length - 1.0f));

        chirpTemplate[n] = chirpValue * window;
    } // end for
    
    float energy = 0.0f;

    for (uint32_t n = 0; n < length; n++)
    {
        energy += chirpTemplate[n] * chirpTemplate[n];
    } // end for

    float norm = sqrtf(energy);

    if (norm > 1e-12)
    {
        for (uint32_t n = 0; n < length; n++)
        {
            chirpTemplate[n] /= norm;
        } // end for
    } // end if
} // end function APP_Chirp_Generate