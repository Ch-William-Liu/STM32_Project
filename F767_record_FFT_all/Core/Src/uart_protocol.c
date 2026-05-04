#include "uart_protocol.h"
#include "adc_audio.h"
#include "fft_process.h"
#include "wav_recorder.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart3;

#define UART_RX_BUF_SIZE 128
#define UART_TX_BUF_SIZE 256

static uint8_t rx_byte;
static char cmd_buf[UART_RX_BUF_SIZE];
static uint32_t cmd_idx = 0;

void UART_Protocol_Init(void)
{
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
} // end function UART_Protocol_Init

void UART_SendText(const char *text)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)text, strlen(text), HAL_MAX_DELAY);
} // end function UART_SendText

static void UART_SendLine(const char *text)
{
    UART_SendText(text);
    UART_SendText("\r\n");
} // end function UART_SendLine

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (rx_byte == '\n' || rx_byte == '\r')
        {
            if (cmd_idx > 0)
            {
                cmd_buf[cmd_idx] = '\0';
                cmd_idx = 0;
            } // end if
        } // end if
        else
        {
            if (cmd_idx < UART_RX_BUF_SIZE - 1)
            {
                cmd_buf[cmd_idx++] = rx_byte;
            } // end if
        } // end else
        
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    } // end if 
} // end function HAL_UART_RxCpltCallback

static uint8_t IsValidFS(uint32_t fs)
{
    return fs == 16000 || fs == 32000 || fs == 64000 || fs == 100000;
} // end function IsValidFS

static uint8_t IsValidFFT(uint32_t fft)
{
    return fft == 1024 || fft == 2048 || fft == 4096;
} // end function IsValidFFT

static void ProcessCommand(char *cmd)
{
    uint32_t fs;
    uint32_t fft;
    uint32_t duration;

    if (sscanf(cmd, "SET FS=%lu FFT=%lu", &fs, &fft) == 2)
    {
        if (!IsValidFS(fs))
        {
            UART_SendLine("ERR INVALID FS");
            return;
        } // end if valid fs
        
        if (!IsValidFFT(fft))
        {
            UART_SendLine("ERR INVALID FFT");
            return;
        } // end if valid fft

        ADC_Audio_Stop();

        ADC_Audio_SetSampleRate(fs);
        FFT_Process_SetConfig(fs, fft);

        ADC_Audio_Start();

        char msg[96];
        sprintf(msg, "OK SET FS=%lu FFT=%lu", fs, fft);
        UART_SendLine(msg);

        return;
    } // end if

    if (sscanf(cmd, "REC %lu", &duration) == 1)
    {
        if (duration == 0)
        {
            UART_SendLine("ERR INVALID REC TIME");
            return;
        } // end if duration = 0

        WAV_Recorder_Start(duration, ADC_Audio_GetSampleRate());
        return;
    } // end sscanf ==  1

    if (strcmp(cmd , "STOP_REC") == 0)
    {
        WAV_Recorder_Stop();
        return;
    } // end if stop rec

    if (strcmp(cmd, "STATUS") == 0)
    {
        char msg[128];

        sprintf(
            msg,
            "STATUS FS=%lu FFT=%lu REC+%u",
            ADC_Audio_GetSampleRate(),
            FFT_Process_GetSize(),
            WAV_Recorder_IsRecording()
        );

        UART_SendLine(msg);
        return;
    } // end if status
    
    UART_SendLine("ERR UNKNOWN CMD");

} //end function ProcessCommand

void UART_Protocol_Process(void)
{
    static char last_cmd[UART_RX_BUF_SIZE];

    if (cmd_buf[0] != '\0')
    {
        __disable_irq();

        strncpy(last_cmd, cmd_buf, UART_RX_BUF_SIZE);
        cmd_buf[0] = '\0';

        __enable_irq();

        ProcessCommand(last_cmd);
    } // end if
} // end function UART_Protocol_Process

void UART_SendFFTFrame(
    uint32_t frame_index,
    uint32_t fs,
    uint32_t fft_size,
    float *mag,
    uint32_t bin_count
)
{
    /*
    Binary frame format:
        0xAA 0x55
        frame_index     uint32
        fs              uint32
        fft_size        uint32
        bin_count       uint32
        payload         uint16_t magnitude[bit_count]
        0x55 0xAA
    */

    uint8_t header[18];
    uint8_t tail[2] = {0x55, 0xAA};

    header[0] = 0xAA;
    header[1] = 0x55;

    memcpy(&header[2], &frame_index, 4);
    memcpy(&header[6], &fs, 4);
    memcpy(&header[10], &fft_size, 4);
    memcpy(&header[14], &bin_count, 4);

    HAL_UART_Transmit(&huart3, header, 18, HAL_MAX_DELAY);

    for (uint32_t i = 0; i < bin_count; i++)
    {
        uint16_t value;

        if (mag[i] < 0)
        {
            value = 0;
        } // end if
        else if (mag[i] > 65535.0f)
        {
            value = 65535;
        } // end else if
        else
        {
            value = (uint16_t)mag[i];
        } // end else

        HAL_UART_Transmit(&huart3, (uint8_t*)&value, 2, HAL_MAX_DELAY);
    } // end for
    
    HAL_UART_Transmit(&huart3, tail, 2, HAL_MAX_DELAY);
} // end function UART_SendFFTFrame