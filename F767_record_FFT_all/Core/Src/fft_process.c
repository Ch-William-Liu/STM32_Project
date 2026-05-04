#include "fft_process.h"
#include "uart_protocol.h"
#include "main.h"

#include "arm_math.h"

#include <math.h>
#include <string.h>
#include <stdint.h>

static uint32_t current_fs = 32000;
static uint32_t current_fft_size = 2048;

static float32_t fft_input[FFT_MAX_SIZE];
static float32_t fft_output[FFT_MAX_SIZE];
static float32_t fft_mag[FFT_MAX_SIZE / 2];

static float32_t fft_accum[FFT_MAX_SIZE];
static uint32_t fft_accum_count = 0;

static arm_rfft_fast_instance_f32 rfft_instance;

static uint32_t frame_index = 0;

void FFT_Process_Init(uint32_t fs, uint32_t fft_size)
{
    FFT_Process_SetConfig(fs, fft_size);
} // end function FFT_Process_Init

void FFT_Process_SetConfig(uint32_t fs, uint32_t fft_size)
{
    if (fft_size > FFT_MAX_SIZE)
    {
        fft_size = FFT_MAX_SIZE;
    } // end if

    if (!(fft_size == 1024 || fft_size == 2048 || fft_size == 4096))
    {
        fft_size = 2048;
    } // end if

    current_fs = fs;
    current_fft_size = fft_size;
    fft_accum_count = 0;

    arm_rfft_fast_init_f32(&rfft_instance, current_fft_size);
} // end function FFT_Process_SetConfig

uint32_t FFT_Process_GetSize(void)
{
    return current_fft_size;
} // end function FFT_Process_GetSize

uint32_t FFT_Process_GetSampleRate(void)
{
    return current_fs;
} // end function FFT_Process_GetSampleRate

static float32_t HannWindow(uint32_t n, uint32_t N)
{
    return 0.5f - 0.5f * arm_cos_f32((2.0f * PI * n) / (N - 1));
} // end function HannWindow

static void RunFFT(void)
{
    float32_t mean = 0.0f;

    for (uint32_t i = 0; i < current_fft_size; i++)
    {
        mean += fft_accum[i];
    } // end for

    mean /= current_fft_size;

    for (uint32_t i = 0; i < current_fft_size; i++)
    {
        float32_t x = fft_accum[i] - mean;
        fft_input[i] = x * HannWindow(i, current_fft_size);
    } // end for

    arm_rfft_fast_f32(&rfft_instance, fft_input, fft_output, 0);

    fft_mag[0] = fabsf(fft_output[0]);

    for (uint32_t i = 1; i < current_fft_size / 2; i++)
    {
        float32_t real = fft_output[2 * i];
        float32_t imag = fft_output[2 * i + 1];

        fft_mag[i] = sqrtf(real * real + imag * imag);
    } // end for
    
    UART_SendFFTFrame(frame_index, current_fs, current_fft_size, fft_mag, current_fft_size / 2);

    frame_index++;
} // end function RunFFT

void FFT_Process_Frame(uint16_t *adc_buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        float32_t sample = (float32_t)adc_buf[i];

        fft_accum[fft_accum_count++] = sample;

        if (fft_accum_count >= current_fft_size)
        {
            RunFFT();
            fft_accum_count = 0;
        } // end if
    } // end for
    
} // end function FFT_Process_Frame