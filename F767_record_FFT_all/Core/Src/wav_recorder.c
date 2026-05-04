#include "wav_recorder.h"
#include "uart_protocol.h"
#include "fatfs.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

static FATFS fs;
static FIL wav_file;

static uint8_t fs_mounted = 0;
static uint8_t is_recording = 0;

static uint32_t target_samples = 0;
static uint32_t written_samples = 0;
static uint32_t current_sample_rate = 32000;

static uint32_t file_index = 1;

static void WriteWavHeader(FIL *file, uint32_t sample_rate, uint32_t data_bytes)
{
    uint8_t header[44];

    uint32_t byte_rate = sample_rate * 1 * 16 / 8;
    uint32_t block_align = 1 * 16 / 8;
    uint32_t chunk_size = 36 + data_bytes;

    uint32_t subchunk1_size = 16;
    uint16_t audio_format = 1;
    uint16_t num_channels = 1;
    uint16_t bits_per_sample = 16;

    memcpy(&header[0], "RIFF", 4);
    memcpy(&header[4], &chunk_size, 4);
    memcpy(&header[8], "WAVE", 4);
    
    memcpy(&header[12], "fmt", 4);
    memcpy(&header[16], &subchunk1_size, 4);
    memcpy(&header[20], &audio_format, 2);
    memcpy(&header[22], &num_channels, 2);
    memcpy(&header[24], &sample_rate, 4);
    memcpy(&header[28], &byte_rate, 4);
    memcpy(&header[32], &block_align, 2);
    memcpy(&header[34], &bits_per_sample, 2);

    memcpy(&header[36], "data", 4);
    memcpy(&header[40], &data_bytes, 4);

    UINT bw;
    f_lseek(file, 0);
    f_write(file, header, 44, &bw);
} // end function WriteWavHeader

void WAV_Recorder_Init(void)
{
    FRESULT res;

    is_recording = 0;

    res = f_mount(&fs, "", 1);

    if (res == FR_OK)
    {
        fs_mounted = 1;
        UART_SendText("OK SD MOUNT\r\n");
    } // end if
    else
    {
        fs_mounted = 0;
        UART_SendText("ERR SD MOUNT\r\n");
    } // end else
} // end function WAV_Recorder_Init

uint8_t WAV_Recorder_Start(uint32_t duration_sec, uint32_t sample_rate)
{
    char filename[32];
    UINT bw;
    FRESULT res;

    if (!fs_mounted)
    {
        UART_SendText("ERR SD NOT MOUNTED\r\n");
        return 0;
    } // end if

    if (is_recording)
    {
        UART_SendText("ERR REC BUSY\r\n");
        return 0;
    } // end if

    current_sample_rate = sample_rate;
    target_samples = duration_sec * sample_rate;
    written_samples = 0;

    sprintf(filename, "REC_%04lu.WAV", file_index++);

    res = f_open(&wav_file, filename, FA_CREATE_ALWAYS | FA_WRITE);

    if (res != FR_OK)
    {
        UART_SendText("ERR SD OPEN FAIL\r\n");
        return 0;
    } // end if

    uint8_t empty_header[44] = {0};
    f_write(&wav_file, empty_header, 44, &bw);
    
    is_recording = 1;

    UART_SendText("OK REC START\r\n");

    return 1;
} // end function WAV_Recorder_Start

void WAV_Recorder_Stop(void)
{
    if (!is_recording)
    {
        return;
    } // end if

    uint32_t data_bytes = written_samples * 2;
    WriteWavHeader(&wav_file, current_sample_rate, data_bytes);

    f_close(&wav_file);

    is_recording = 0;

    UART_SendText("OK REC DONE\r\n");
} // end function WAV_Recorder_Stop

void WAV_Recorder_Process(void)
{
    if (is_recording && written_samples >= target_samples)
    {
        WAV_Recorder_Stop();
    } // end if
} // end function WAV_Recorder_Process

void WAV_Recorder_WriteADC(uint16_t *adc_buf, uint32_t len)
{
    if (!is_recording)
    {
        return;
    } // end function is_recording
    
    int16_t pcm_buf[512];
    UINT bw;

    uint32_t remain = target_samples - written_samples;
    uint32_t samples_to_write = len;

    if (samples_to_write > remain)
    {
        samples_to_write = remain;
    } // end if

    uint32_t idx = 0;

    while (idx < samples_to_write)
    {
        uint32_t block = samples_to_write - idx;

        if (block > 512)
        {
            block = 512;
        } // end if

        for (uint32_t i = 0; i < block; i++)
        {
            int32_t centered = (int32_t)adc_buf[idx + i] - 2048;
            pcm_buf[i] = (int16_t)(centered<<4);
        } // end for

        if (f_write(&wav_file, pcm_buf, block * sizeof(int16_t), &bw) != FR_OK)
        {
            UART_SendText("ERR SD WRITE FAIL\r\n");
            WAV_Recorder_Stop();
            return;
        } // end if
        idx += block;
        written_samples += block;
    } // end while

    if (written_samples >= target_samples)
    {
        WAV_Recorder_Stop();
    } // end if

} // end function WAV_Recorder_WriteADC

uint8_t WAV_Recorder_IsRecording(void)
{
    return is_recording;
}