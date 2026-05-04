#include "app_main.h"
#include "adc_audio.h"
#include "fft_process.h"
#include "wav_recorder.h"
#include "uart_protocol.h"
#include <stdint.h>

static volatile uint8_t adc_half_ready = 0;
static volatile uint8_t adc_full_ready = 0;

void APP_Init(void)
{
    UART_Protocol_Init();
    UART_SendText("BOOT 1 UART INIT OK\r\n");
    ADC_Audio_Init();
    UART_SendText("BOOT 2 AUDIO INIT OK\r\n");
    FFT_Process_Init(32000, 2048);
    UART_SendText("BOOT 3 FFT INIT OK\r\n");

    WAV_Recorder_Init();
    UART_SendText("BOOT 4 WAV recorder INIT OK\r\n");
    ADC_Audio_Start();
    UART_SendText("System Ready\r\n");
} // end function APP_Init

void APP_Loop(void)
{
    UART_Protocol_Process();

    if (adc_half_ready)
    {
        adc_half_ready = 0;
        ADC_Audio_ProcessHalf();
    } // end if

    if (adc_full_ready)
    {
        adc_full_ready = 0;
        ADC_Audio_ProcessFull();
    } // end if

    WAV_Recorder_Process();
} // end function APP_Loop

void APP_ADC_HalfCallback(void)
{
    adc_half_ready = 1;
} // end function APP_ADC_HalfCallback

void APP_ADC_FullCallback(void)
{
    adc_full_ready = 1;
} // end function APP_ADC_FullCallback