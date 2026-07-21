#include "dac_stream.h"
#include "dbpsk_mod.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define PI 3.14159265358979323846f

extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim6;

#if defined(__GNUC__)
__attrubute__((aligned(32)))
#endif
static uint16_t stream_buffer[DAC_STREAM_BUFFER_SIZE];

static volatile uint32_t sync_sample_index = 0U;
static volatile uint8_t stream_running = 0U;
static volatile uint8_t stream_error = 0U;

static uint32_t GetSyncTotalSamples(void)
{
  return (uint32_t)(DAC_FS * SYNC_CHIRP_DURATION + 0.5f);
} // end function GetSyncTotalSamples

static uint16_t GenerateSyncSample(uint32_t global_index)
{
  const uint32_t total_samples = GetSyncTotalSamples();

  if (total_samples < 2U)
  {
    return DAC_MID;
  } // end if total samples too small

  /* If length > total chirp length, restart */
  const uint32_t n = global_index % total_samples;

  const float duration = SYNC_CHIRP_DURATION;
  const float t = (float)n / DAC_FS;

  const float f_start = SYNC_CHIRP_START_FREQ;
  const float f_end   = SYNC_CHIRP_END_FREQ;

  const float chirp_rate = (f_end - f_start) / duration;

  const float phase = 2.0f * PI * (f_start * t + 0.5f * chirp_rate * t * t);

  const float window = 0.5f * (1.0f - cosf(2.0f * PI * (float)n / (float)(total_samples - 1U)));
  const float signal = sinf(phase) * window;

  int32_t dac_value = DAC_MID + (int32_t)(DAC_AMP * signal);

  if (dac_value < 0)
  {
    dac_value = 0;
  } // end if dac value too small
  else if (dac_value > 4095)
  {
    dac_value = 4095;
  } // end if dac value too big

  return (uint16_t)dac_value;
} // end function GenerateSyncSample

static void FillStreamBuffer(uint32_t offset, uint32_t length)
{
  if ((offset + length) > DAC_STREAM_BUFFER_SIZE)
  {
    stream_error = 1U;
    return;
  } // end if offset and length too big

  for (uint32_t i = 0U; i < length; i++)
  {
    stream_buffer[offset + i] = GenerateSyncSample(sync_sample_index);

    sync_sample_index++;
  } // end for
} // end function FillStreamBuffer

HAL_StatusTypeDef DAC_Stream_Start(void)
{
  HAL_StatusTypeDef status;

  if (stream_running != 0U)
  {
    return HAL_BUSY;
  } // end if stream is running

  stream_error = 0U;
  sync_sample_index = 0U;

  FillStreamBuffer(0U, DAC_STREAM_HALF_SIZE);

  FillStreamBuffer(DAC_STREAM_HALF_SIZE , DAC_STREAM_HALF_SIZE);

  if (stream_error != 0U)
  {
    return HAL_ERROR;
  } // end if is error

  status = HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)stream_buffer, DAC_STREAM_BUFFER_SIZE, DAC_ALIGN_12B_R);

  if (status != HAL_OK)
  {
    return status;
  } // end if status not OK

  status = HAL_TIM_Base_Start(&htim6);

  if (status != HAL_OK)
  {
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    return status;
  } // end if status not OK

  stream_running = 1U;

  return HAL_OK;
} // end function DAC_Stream_Start

void DAC_Stream_Stop(void)
{
  stream_running = 0U;

  HAL_TIM_Base_Stop(&htim6);
  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);

  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_MID);
} // end function DAC_Stream_Stop

uint8_t DAC_Stream_IsRunning(void)
{
  return stream_running;
} // end function DAC_Stream_IsRunning

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac_handle)
{
  if ((hdac_handle == NULL) || (hdac_handle->Instance != DAC) || (stream_running == 0U))
  {
    return;
  } // end if

  FillStreamBuffer(0U, DAC_STREAM_HALF_SIZE);
} // end function HAL_DAC_ConvHalfCpltCallbackCh1

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac_handle)
{
  if ((hdac_handle == NULL) || (hdac_handle->Instance != DAC) || (stream_running == 0U))
  {
    return;
  } // end if

  FillStreamBuffer(DAC_STREAM_HALF_SIZE, DAC_STREAM_HALF_SIZE);
} // end function HAL_DAC_ConvCpltCallbackCh1

void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac_handle)
{
  if ((hdac_handle != NULL) && (hdac_handle->Instance == DAC))
  {
    stream_error = 1U;
    stream_running = 0U;
  } // end if
} // end function HAL_DAC_ErrorCallbackCh1