#include "app_utils.h"
#include <math.h>

void APP_Int16ToFloat(const int16_t *input, float *output, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        output[i] = (float)input[i] / 32769.0f;
    } // end for
    
} // end function APP_Int16ToFloat

void APP_RemoveDC(float *x, uint32_t length)
{
    float sum = 0.0f;
    for (uint32_t i = 0; i < length; i++)
    {
        sum += x[i];
    } // end for

    float mean = sum / (float)length;

    for (uint32_t i = 0; i < length; i++)
    {
        x[i] -= mean;
    } // end for
} // end function APP_RemoveDC

void APP_NormalizeFrame(float *x, uint32_t length)
{
    float energy = 0.0f;

    for (uint32_t i = 0; i < length; i++)
    {
        energy += x[i] * x[i];
    } // end for

    float norm = sqrtf(energy);

    if (norm > 1e-12f)
    {
        for (uint32_t i = 0; i < length; i++)
        {
            x[i] /= norm;
        } // end for
    } // end if
    
} // end function APP_NormalizeFrame