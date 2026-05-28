#ifndef APP_WAV_H
#define APP_WAV_H

#include <stdint.h>
#include "ff.h"

typedef struct 
{
    FIL file;

    uint32_t sampleRate;
    uint16_t numChannels;
    uint16_t bitsPerSample;

    uint32_t dataStartOffset;
    uint32_t dataSizeBytes;
    uint32_t currentByteOffset;
} APP_WavFile;

int APP_Wav_Open(APP_WavFile *wav, const char *filename);
int APP_Wav_ReadSamples(APP_WavFile *wav, int16_t *buffer, uint32_t sampleCount, uint32_t *samplesRead);
void APP_Wav_Close(APP_WavFile *wav);

#endif