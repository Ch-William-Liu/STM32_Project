#include "adc_audio.h"
#include "fft_process.h"
#include "wav_recorder.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;

uint16_t adc_dma_buf[ADC_DMA_SIZE];

static uint32_t current_fs = 32000;

void ADC_Audio_Init(void)
{
    ADC_Audio_SetSampleRate(current_fs);
} // end function ADC_Audio_Init

void ADC_Audio_Start(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, ADC_DMA_SIZE);
    HAL_TIM_Base_Start(&htim2);
} // end function ADC_Audio_Start

void ADC_Audio_Stop(void)
{
    HAL_TIM_Base_Stop(&htim2);
    HAL_ADC_Stop_DMA(&hadc1);
} // end function ADC_Audio_Stop

void ADC_Audio_SetSampleRate(uint32_t fs)
{
    uint32_t tim_clk = 108000000;
    uint32_t arr;

    if (fs == 0)
    {
        return;
    } // end if

    current_fs = fs;
    arr = (tim_clk / fs) - 1;

    HAL_TIM_Base_Stop(&htim2);

    __HAL_TIM_SET_PRESCALER(&htim2, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    HAL_TIM_Base_Start(&htim2);
} // end ADC_Audio_SetSampleRate

uint32_t ADC_Audio_GetSampleRate(void)
{
    return current_fs;
} // end function ADC_Audio_GetSampleRate

static void ProcessFrame(uint16_t *buf, uint32_t len)
{
    FFT_Process_Frame(buf, len);
    WAV_Recorder_WriteADC(buf, len);
} // end function ProcessFrame

void ADC_Audio_ProcessHalf(void)
{
    ProcessFrame(&adc_dma_buf[0], ADC_DMA_SIZE / 2);
} // end function ADC_Audio_ProcessHalf

void ADC_Audio_ProcessFull(void)
{
    ProcessFrame(&adc_dma_buf[ADC_DMA_SIZE / 2], ADC_DMA_SIZE / 2);
} // end function ADC_Audio_ProcessFull