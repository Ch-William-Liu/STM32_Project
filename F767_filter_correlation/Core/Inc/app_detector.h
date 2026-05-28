#ifndef APP_DETECTOR_H
#define APP_DETECTOR_H

#include <stdint.h>

typedef struct
{
    float peakValue;
    uint32_t peakIndex;
    uint8_t detected;
} APP_DetectroResult;

void APP_Detector_Correlate(const float *frame, const float *chirpTemplate, float *corr);
APP_DetectroResult APP_Detector_FindPeak(const float *corr, uint32_t corrLen, float threshold);

#endif