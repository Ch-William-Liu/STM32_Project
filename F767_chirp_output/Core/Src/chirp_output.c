#include "chirp_output.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define PI_F                        3.14159265358979323846f
#define CHIRP_TOTAL_SAMPLES         ((uint32_t)(CHIRP_DAC_FS_HZ * CHIRP_DURATION_SEC + 0.5f))

#if defined(__GNUC__)
__attribute__((aligned(32)))
#endif
static uint16_t stream_buffer[CHIRP_STREAM_BUFFER_SIZE];

static DAC_HandleTypeDef *player_hdac = NULL;
static TIM_HandleTypeDef *player_htim = NULL;

static volatile uint8_t fill_first_half_request = 0U;
static volatile uint8_t fill_second_half_request = 0U;
static volatile uint8_t stream_running = 0U;
static volatile uint8_t stream_finished = 0U;
static volatile uint8_t stream_error = 0U;

static uint32_t generated_sample_count = 0U;
static volatile uint32_t played_sample_count = 0U;

static uint16_t FloatToDAC(float signal)
{
  float dac_value = (float)CHIRP_DAC_MID + CHIRP_DAC_AMP * signal;

  if (dac_value < 0.0f)
  {
    dac_value = 0.0f;
  }
  else if (dac_value > 4095.0f)
  {
    dac_value = 4095.0f;
  }

  return (uint16_t)(dac_value + 0.5f);
}

static uint16_t GenerateChirpSample(uint32_t n)
{
  if ((CHIRP_TOTAL_SAMPLES < 2U) || (n >= CHIRP_TOTAL_SAMPLES))
  {
    return (uint16_t)CHIRP_DAC_MID;
  }

  const float t = (float)n / (float)CHIRP_DAC_FS_HZ;
  const float chirp_rate =
      (CHIRP_END_FREQ_HZ - CHIRP_START_FREQ_HZ) / CHIRP_DURATION_SEC;

  const float phase = 2.0f * PI_F *
      (CHIRP_START_FREQ_HZ * t + 0.5f * chirp_rate * t * t);

  const float window = 0.5f *
      (1.0f - cosf(2.0f * PI_F * (float)n /
                   (float)(CHIRP_TOTAL_SAMPLES - 1U)));

  return FloatToDAC(sinf(phase) * window);
}

static void FillStreamBuffer(uint32_t offset, uint32_t length)
{
  if ((offset + length) > CHIRP_STREAM_BUFFER_SIZE)
  {
    stream_error = 1U;
    return;
  }

  for (uint32_t i = 0U; i < length; i++)
  {
    if (generated_sample_count < CHIRP_TOTAL_SAMPLES)
    {
      stream_buffer[offset + i] = GenerateChirpSample(generated_sample_count);
      generated_sample_count++;
    }
    else
    {
      stream_buffer[offset + i] = (uint16_t)CHIRP_DAC_MID;
    }
  }
}

void ChirpPlayer_Init(DAC_HandleTypeDef *dac_handle,
                      TIM_HandleTypeDef *tim_handle)
{
  player_hdac = dac_handle;
  player_htim = tim_handle;

  fill_first_half_request = 0U;
  fill_second_half_request = 0U;
  stream_running = 0U;
  stream_finished = 0U;
  stream_error = ((dac_handle == NULL) || (tim_handle == NULL)) ? 1U : 0U;

  generated_sample_count = 0U;
  played_sample_count = 0U;

  for (uint32_t i = 0U; i < CHIRP_STREAM_BUFFER_SIZE; i++)
  {
    stream_buffer[i] = (uint16_t)CHIRP_DAC_MID;
  }
}

HAL_StatusTypeDef ChirpPlayer_Play(void)
{
  if ((player_hdac == NULL) || (player_htim == NULL))
  {
    stream_error = 1U;
    return HAL_ERROR;
  }

  if (stream_running != 0U)
  {
    return HAL_BUSY;
  }

  fill_first_half_request = 0U;
  fill_second_half_request = 0U;
  stream_running = 0U;
  stream_finished = 0U;
  stream_error = 0U;
  generated_sample_count = 0U;
  played_sample_count = 0U;

  FillStreamBuffer(0U, CHIRP_STREAM_HALF_SIZE);
  FillStreamBuffer(CHIRP_STREAM_HALF_SIZE, CHIRP_STREAM_HALF_SIZE);

  if (stream_error != 0U)
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = HAL_DAC_Start_DMA(
      player_hdac,
      DAC_CHANNEL_1,
      (uint32_t *)stream_buffer,
      CHIRP_STREAM_BUFFER_SIZE,
      DAC_ALIGN_12B_R);

  if (status != HAL_OK)
  {
    stream_error = 1U;
    return status;
  }

  status = HAL_TIM_Base_Start(player_htim);

  if (status != HAL_OK)
  {
    (void)HAL_DAC_Stop_DMA(player_hdac, DAC_CHANNEL_1);
    stream_error = 1U;
    return status;
  }

  stream_running = 1U;
  return HAL_OK;
}

void ChirpPlayer_Process(void)
{
  if (stream_running == 0U)
  {
    return;
  }

  if (fill_first_half_request != 0U)
  {
    fill_first_half_request = 0U;
    FillStreamBuffer(0U, CHIRP_STREAM_HALF_SIZE);
  }

  if (fill_second_half_request != 0U)
  {
    fill_second_half_request = 0U;
    FillStreamBuffer(CHIRP_STREAM_HALF_SIZE, CHIRP_STREAM_HALF_SIZE);
  }

  if ((played_sample_count >= CHIRP_TOTAL_SAMPLES) || (stream_error != 0U))
  {
    ChirpPlayer_Stop();

    if (stream_error == 0U)
    {
      stream_finished = 1U;
    }
  }
}

void ChirpPlayer_Stop(void)
{
  if ((player_hdac != NULL) && (player_htim != NULL))
  {
    (void)HAL_TIM_Base_Stop(player_htim);
    (void)HAL_DAC_Stop_DMA(player_hdac, DAC_CHANNEL_1);
    (void)HAL_DAC_SetValue(player_hdac,
                           DAC_CHANNEL_1,
                           DAC_ALIGN_12B_R,
                           CHIRP_DAC_MID);
  }

  stream_running = 0U;
  fill_first_half_request = 0U;
  fill_second_half_request = 0U;
}

uint8_t ChirpPlayer_IsRunning(void)
{
  return stream_running;
}

uint8_t ChirpPlayer_IsFinished(void)
{
  return stream_finished;
}

uint8_t ChirpPlayer_HasError(void)
{
  return stream_error;
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac_handle)
{
  if ((hdac_handle == NULL) || (player_hdac == NULL) ||
      (hdac_handle->Instance != player_hdac->Instance) ||
      (stream_running == 0U))
  {
    return;
  }

  played_sample_count += CHIRP_STREAM_HALF_SIZE;
  fill_first_half_request = 1U;
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac_handle)
{
  if ((hdac_handle == NULL) || (player_hdac == NULL) ||
      (hdac_handle->Instance != player_hdac->Instance) ||
      (stream_running == 0U))
  {
    return;
  }

  played_sample_count += CHIRP_STREAM_HALF_SIZE;
  fill_second_half_request = 1U;
}

void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac_handle)
{
  if ((hdac_handle == NULL) || (player_hdac == NULL) ||
      (hdac_handle->Instance != player_hdac->Instance))
  {
    return;
  }

  stream_error = 1U;
}
