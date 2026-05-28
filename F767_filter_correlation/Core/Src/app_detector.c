#include "app_detector.h"
#include "app_config.h"
#include <math.h>

void APP_Detector_Correlate(const float *frame, const float *chirpTemplate, float *corr)
{
    for (uint32_t k = 0; k < APP_CORR_LEN; k++)
    {
        float sum = 0.0f;
        for (uint32_t n = 0; n < APP_CHIRP_LEN; n++)
        {
            sum += frame[k + n] * chirpTemplate[n];
        } // end inner for

        corr[k] = sum;
    } // end for
} // end function APP_Detector_Correlate

APP_DetectroResult APP_Detector_FindPeak(const float *corr, uint32_t corrLen, float threshold)
{
    APP_DetectroResult result;

    result.peakValue = 0.0f;
    result.peakIndex = 0;
    result.detected = 0;

    for (uint32_t i = 0; i < corrLen; i++)
    {
        float v = fabsf(corr[i]);

        if (v > result.peakValue)
        {
            result.peakValue = v;
            result.peakIndex = i;
        } // end if
    } // end for

    if (result.peakValue >= threshold)
    {
        result.detected = 1;
    } // end if

    return result;
} // end function APP_Dector_FindPeak