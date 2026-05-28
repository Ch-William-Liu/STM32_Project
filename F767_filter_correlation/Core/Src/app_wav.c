#include "app_wav.h"
#include <string.h>

static uint32_t read_u32_le(uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static uint16_t read_u16_le(uint8_t *p)
{
    return ((uint16_t)p[0]) |
           ((uint16_t)p[1] << 8);
}

int APP_Wav_Open(APP_WavFile *wav, const char *filename)
{
    FRESULT res;
    UINT br;
    uint8_t header[44];

    memset(wav, 0, sizeof(APP_WavFile));

    res = f_open(&wav->file, filename, FA_READ);

    if (res != FR_OK)
    {
        return -1;
    }

    res = f_read(&wav->file, header, 44, &br);

    if (res != FR_OK || br != 44)
    {
        f_close(&wav->file);
        return -2;
    }

    if (memcmp(&header[0], "RIFF", 4) != 0 ||
        memcmp(&header[8], "WAVE", 4) != 0 ||
        memcmp(&header[12], "fmt ", 4) != 0)
    {
        f_close(&wav->file);
        return -3;
    }

    uint16_t audioFormat = read_u16_le(&header[20]);

    wav->numChannels   = read_u16_le(&header[22]);
    wav->sampleRate    = read_u32_le(&header[24]);
    wav->bitsPerSample = read_u16_le(&header[34]);

    if (audioFormat != 1)
    {
        f_close(&wav->file);
        return -4;
    }

    if (wav->numChannels != 1)
    {
        f_close(&wav->file);
        return -5;
    }

    if (wav->bitsPerSample != 16)
    {
        f_close(&wav->file);
        return -6;
    }

    /*
     * Simple WAV parser.
     * This assumes standard PCM WAV with data chunk at byte 44.
     */
    if (memcmp(&header[36], "data", 4) != 0)
    {
        f_close(&wav->file);
        return -7;
    }

    wav->dataStartOffset = 44;
    wav->dataSizeBytes = read_u32_le(&header[40]);
    wav->currentByteOffset = 0;

    return 0;
}

int APP_Wav_ReadSamples(APP_WavFile *wav, int16_t *buffer, uint32_t sampleCount, uint32_t *samplesRead)
{
    FRESULT res;
    UINT br;

    uint32_t bytesToRead = sampleCount * sizeof(int16_t);

    if (wav->currentByteOffset >= wav->dataSizeBytes)
    {
        *samplesRead = 0;
        return 0;
    }

    uint32_t remainingBytes = wav->dataSizeBytes - wav->currentByteOffset;

    if (bytesToRead > remainingBytes)
    {
        bytesToRead = remainingBytes;
    }

    res = f_read(&wav->file, buffer, bytesToRead, &br);

    if (res != FR_OK)
    {
        *samplesRead = 0;
        return -1;
    }

    wav->currentByteOffset += br;
    *samplesRead = br / sizeof(int16_t);

    return 0;
}

void APP_Wav_Close(APP_WavFile *wav)
{
    f_close(&wav->file);
}