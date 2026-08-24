#include "dac_stream.h"
#include "dbpsk_mod.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define PI                          3.14159265358979323846f

#define SYNC_CHIRP_REPEAT_COUNT     3U
#define POST_SYNC_SILENCE_SEC       1.0f
#define PAIR_GUARD_DURATION_MS      100.0f

#define CHIRP_DURATION_SEC          0.0035f

#define CHIRP_SAMPLES               ((uint32_t)(DAC_FS * CHIRP_DURATION_SEC + 0.5f))
#define POST_SYMC_SILENCE_SAMPLES   ((uint32_t)(DAC_FS * POST_SYNC_SILENCE_SEC + 0.5f))
#define PAIR_GUARD_SAMPLES          ((uint32_t)(DAC_FS * PAIR_GUARD_DURATION_MS / 1000.0f + 0.5f))

#define SYMBOL_PAIR_COUNT           4U
#define SYMBOL_VALUE_COUNT          4U
#define DRAIN_CALLBACK_TAEGET       2U

extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim6;

#if defined(__GNUC__)
__attribute__((aligned(32)))
#endif
static uint16_t stream_buffer[DAC_STREAM_BUFFER_SIZE];

#if defined(__GNUC__)
__attribute__((aligned(32)))
#endif

static uint16_t symbol_cache[SYMBOL_PAIR_COUNT][SYMBOL_VALUE_COUNT][SAMPLES_PER_SYMBOL];
static uint8_t symbol_cache_ready = 0U;

static volatile uint8_t fill_first_half_request = 0U;
static volatile uint8_t fill_second_half_request = 0U;
static volatile uint8_t stream_running = 0U;
static volatile uint8_t stream_finished = 0U;
static volatile uint8_t stream_error = 0U;

static uint8_t stream_packets[DAC_STREAM_PAIR_COUNT][DAC_STREAM_MAX_PACKET_SIZE];

static uint16_t stream_packet_lengths[DAC_STREAM_PAIR_COUNT];

typedef enum
{
  DAC_STREAM_STATE_IDLE = 0,
  DAC_STREAM_STATE_SYNC_CHIRP,
  DAC_STREAM_STATE_POST_SYNC_SILENCE,
  DAC_STREAM_STATE_PACKET,
  DAC_STREAM_STATE_PAIR_GUARD,
  DAC_STREAM_STATE_DRAIN,
  DAC_STREAM_STATE_FINISHED
} DAC_StreamState_t;

static DAC_StreamState_t stream_state = DAC_STREAM_STATE_IDLE;
static uint32_t state_sample_index = 0U;
static uint8_t sync_chirp_count = 0U;
static uint8_t current_pair_index = 0U;
static volatile uint8_t drain_callback_count = 0U;

/* 
Frequency table
    |-------------------------------------------------------------------------------------------|
    | Frequency Pair    |   Symbol  |   Center Frequency [kHz]  |   Chirp Frequency Range [kHz] |
    |-------------------|-----------|---------------------------|-------------------------------|
    |           1       |       00  |           28.2            |           28.1->28.3          |
    |           1       |       01  |           28.6            |           28.5->28.7          |
    |           1       |       10  |           29.0            |           28.9->29.1          |
    |           1       |       11  |           29.4            |           29.3->29.5          |
    |-------------------|-----------|---------------------------|-------------------------------|
    |           2       |       00  |           29.8            |           29.7->29.9          |
    |           2       |       01  |           30.2            |           30.1->30.3          |
    |           2       |       10  |           30.6            |           30.5->30.7          |
    |           2       |       11  |           31.0            |           30.9->31.1          |
    |-------------------|-----------|---------------------------|-------------------------------|
    |           3       |       00  |           31.4            |           31.3->31.5          |
    |           3       |       01  |           31.8            |           31.7->31.9          |
    |           3       |       10  |           32.2            |           32.1->32.3          |
    |           3       |       11  |           32.6            |           32.5->32.7          |
    |-------------------|-----------|---------------------------|-------------------------------|
    |           4       |       00  |           33.0            |           32.9->33.1          |
    |           4       |       01  |           33.4            |           33.3->33.5          |
    |           4       |       10  |           33.8            |           33.7->33.9          |
    |           4       |       11  |           34.2            |           34.1->34.3          |
    |-------------------|-----------|---------------------------|-------------------------------|
*/
static const float chirp_center_freq_table[4][4] = 
{
  {28200.0f, 28600.0f, 29000.0f, 29400.0f},
  {29800.0f, 30200.0f, 30600.0f, 31000.0f},
  {31400.0f, 31800.0f, 32200.0f, 32600.0f},
  {33000.0f, 33400.0f, 33800.0f, 34200.0f}
};

static uint16_t FloatToDAC(float signal)
{
  int32_t dac_value = DAC_MID + (int32_t)(DAC_AMP * signal);

  if (dac_value < 0)
  {
    dac_value = 0;
  } // end if dac_value too small
  else if (dac_value > 4095)
  {
    dac_value = 4095;
  } // end if dac_value too big

  return (uint16_t)dac_value;
} // end function FloatToDAC

static uint32_t GetSyncChirpSamples(void)
{
  return (uint32_t)(DAC_FS * SYNC_CHIRP_DURATION + 0.5f);
} // end function GetSyncChirpSamples

static uint16_t GenerateSyncChirpSample(uint32_t n)
{
  const uint32_t total_samples = GetSyncChirpSamples();

  if ((total_samples < 2U) || (n >= total_samples))
  {
    return DAC_MID;
  } // end if total sample too big or idx 

  const float t = (float)n / DAC_FS;

  const float chirp_rate = (SYNC_CHIRP_END_FREQ - SYNC_CHIRP_START_FREQ) / SYNC_CHIRP_DURATION;
  const float phase = 2.0f * PI * (SYNC_CHIRP_START_FREQ * t + 0.5f * chirp_rate * t * t);
  const float window = 0.5f * (1.0f - cosf(2.0f * PI * (float)n / (float)(total_samples - 1U)));

  return FloatToDAC(sinf(phase) * window);
} // end function GenerateSyncChirpSample

static void BuildSymbolCacheEntry(uint8_t pair_index, uint8_t symbol)
{
  const float center_freq = chirp_center_freq_table[pair_index][symbol];

  const float f_start = center_freq - 100.0f;
  const float f_end = center_freq + 100.0f;

  const float chirp_duration = (float)CHIRP_SAMPLES / DAC_FS;

  const float chirp_rate = (f_end - f_start) / chirp_duration;

  for (uint32_t n = 0U; n < CHIRP_SAMPLES; n++)
  {
    const float t = (float)n / DAC_FS;
    const float phase = 2.0f * PI * (f_start * t + 0.5f * chirp_rate * t * t);
    const float window = 0.5f * (1.0f - cosf(2.0f * PI * (float)n / (float)(CHIRP_SAMPLES - 1U)));

    symbol_cache[pair_index][symbol][n] = FloatToDAC(sinf(phase) * window);
  } // end for

  for (uint32_t n = CHIRP_SAMPLES; n < SAMPLES_PER_SYMBOL; n++)
  {
    symbol_cache[pair_index][symbol][n] = DAC_MID;
  } // end for
} // function BuildSymbolCacheEntry

static void BuildALLSymbolCaches(void)
{
  if (symbol_cache_ready != 0U)
  {
    return;
  } // end if cache not ready

  for (uint8_t pair = 0U; pair < SYMBOL_VALUE_COUNT; pair++)
  {
    for (uint8_t symbol = 0U; symbol < SYMBOL_VALUE_COUNT; symbol++)
    {
      BuildSymbolCacheEntry(pair, symbol);
    } // end for
  } // end for

  symbol_cache_ready = 1U;
} // end function BuildAllSymbolCaches

void DAC_Stream_Init(void)
{
  BuildALLSymbolCaches();
} // end function DAC_Stream_Init

static uint16_t GetPacketSample(uint8_t pair_index, uint32_t packet_sample)
{
  if (pair_index >= DAC_STREAM_PAIR_COUNT)
  {
    stream_error = 1U;
    return DAC_MID;
  } // end if

  const uint16_t packet_len = stream_packet_lengths[pair_index];
  const uint32_t total_packet_samples = (uint32_t)packet_len * 4U * SAMPLES_PER_SYMBOL;

  if (packet_sample >= total_packet_samples)
  {
    return DAC_MID;
  } // end if

  const uint32_t symbol_number = packet_sample / SAMPLES_PER_SYMBOL;
  const uint32_t sample_in_symbol = packet_sample % SAMPLES_PER_SYMBOL;

  const uint32_t byte_index = symbol_number / 4U;
  const uint32_t symbol_in_byte = symbol_number % 4U;

  if (byte_index >= packet_len)
  {
    stream_error = 1U;
    return DAC_MID;
  } // end if

  const uint8_t byte = stream_packets[pair_index][byte_index];
  const uint8_t shift = (uint8_t)(6U - 2U * symbol_in_byte);

  const uint8_t symbol = (uint8_t)((byte>>shift) & 0x03U);

  return symbol_cache[pair_index][symbol][sample_in_symbol];
} // end function GetPacketSample

static uint32_t GetCurrentPacketTotalSamples(void)
{
  if (current_pair_index >= DAC_STREAM_PAIR_COUNT)
  {
    stream_error = 1U;
    return 0U;
  } // end if

  return (uint32_t)stream_packet_lengths[current_pair_index] * 4U * SAMPLES_PER_SYMBOL;
} // end function GetCurrentPacketTotalSamples

static void AdvanceStateIfNeeded(void)
{
  switch (stream_state)
  {
    case DAC_STREAM_STATE_SYNC_CHIRP:
    {
      if (state_sample_index >= GetSyncChirpSamples())
      {
        state_sample_index = 0U;
        sync_chirp_count++;

        if (sync_chirp_count >= SYNC_CHIRP_REPEAT_COUNT)
        {
          stream_state = DAC_STREAM_STATE_POST_SYNC_SILENCE;
        } // end if
      } // end if
      break;
    } // end case DAC_STREAM_STATE_SYNC_CHIRP

    case DAC_STREAM_STATE_POST_SYNC_SILENCE:
    {
      if (state_sample_index >= POST_SYMC_SILENCE_SAMPLES)
      {
        state_sample_index = 0U;
        current_pair_index = 0U;
        stream_state = DAC_STREAM_STATE_PACKET;
      } // end if 
      break;
    } // end case DAC_STREAM_STATE_POST_SYNC_SILENCE

    case DAC_STREAM_STATE_PACKET:
    {
      if (state_sample_index >= GetCurrentPacketTotalSamples())
      {
        state_sample_index = 0U;

        if (current_pair_index < (DAC_STREAM_PAIR_COUNT - 1U))
        {
          stream_state = DAC_STREAM_STATE_PAIR_GUARD;
        } // end if
        else
        {
          drain_callback_count = 0U;
          stream_state = DAC_STREAM_STATE_DRAIN;
        } // end else
      } // end if
      break;
    } // end case DAC_STREAM_STATE_PACKET

    case DAC_STREAM_STATE_PAIR_GUARD:
    {
      if (state_sample_index >= PAIR_GUARD_SAMPLES)
      {
        state_sample_index = 0U;
        current_pair_index++;
        stream_state = DAC_STREAM_STATE_PACKET;
      } // end if
      break;
    } // end case DAC_STREAM_STATE_PAIR_GUARD

    default:
    {
      break;
    } // end defaule
  } // ebd switch
} // end function AdvanceStateIfNeeded

static uint16_t GenerateNextSample(void)
{
  AdvanceStateIfNeeded();

  switch (stream_state)
  {
    case DAC_STREAM_STATE_SYNC_CHIRP:
    {
      const uint16_t sample = GenerateSyncChirpSample(state_sample_index);

      state_sample_index++;

      return sample;
    } // end case DAC_STREAM_STATE_SYMC_CHIRP

    case DAC_STREAM_STATE_POST_SYNC_SILENCE:
    {
      state_sample_index++;
      return DAC_MID;
    } // end case DAC_STREAM_STATE_POST_SYNC_SILENCE

    case DAC_STREAM_STATE_PACKET:
    {
      const uint16_t sample = GetPacketSample(current_pair_index, state_sample_index);
      state_sample_index++;

      return sample;
    }  // end case DAC_STREAM_STATE_PACKET

    case DAC_STREAM_STATE_PAIR_GUARD:
    {
      state_sample_index++;
      return DAC_MID;
    } // end case DAC_STREAM_STATE_PAIR_GUARD

    case DAC_STREAM_STATE_DRAIN:
    case DAC_STREAM_STATE_FINISHED:
    case DAC_STREAM_STATE_IDLE:
    default:
    {
      return DAC_MID;
    } // end default
  } // end switch
} // end function GenerateNextSample

static void FillStreamBuffer(uint32_t offset, uint32_t length)
{
  if ((offset + length) > DAC_STREAM_BUFFER_SIZE)
  {
    stream_error = 1U;
    return;
  } // end if

  for (uint32_t i = 0U; i < length; i++)
  {
    stream_buffer[offset + i] = GenerateNextSample();
  } // end for

  SCB_CleanDCache_by_Addr((uint32_t *)&stream_buffer[offset], (int32_t)(length * sizeof(stream_buffer[0])));
} // end function static void FillStreamBuffer

HAL_StatusTypeDef DAC_Stream_StartSequence(const uint8_t packets[DAC_STREAM_PAIR_COUNT][DAC_STREAM_MAX_PACKET_SIZE], const uint16_t packet_lengths[DAC_STREAM_PAIR_COUNT])
{
  if ((packets == NULL) || (packet_lengths == NULL))
  {
    return HAL_ERROR;
  } // end if

  if (stream_running != 0U)
  {
    return HAL_BUSY;
  } // end if

  BuildALLSymbolCaches();

  for (uint8_t pair = 0U; pair < DAC_STREAM_PAIR_COUNT; pair++)
  {
    if ((packet_lengths[pair] == 0U) || (packet_lengths[pair] > DAC_STREAM_MAX_PACKET_SIZE))
    {
      return HAL_ERROR;
    } // end if

    stream_packet_lengths[pair] = packet_lengths[pair];

    memcpy(stream_packets[pair], packets[pair], packet_lengths[pair]);
  } // end for

  fill_first_half_request = 0U;
  fill_second_half_request = 0U;

  stream_error = 0U;
  stream_finished = 0U;
  stream_running = 0U;

  sync_chirp_count = 0U;
  current_pair_index = 0U;
  state_sample_index = 0U;
  drain_callback_count = 0U;

  stream_state = DAC_STREAM_STATE_SYNC_CHIRP;

  FillStreamBuffer(0U, DAC_STREAM_HALF_SIZE);
  FillStreamBuffer(DAC_STREAM_HALF_SIZE, DAC_STREAM_HALF_SIZE);

  if (stream_error != 0U)
  {
    stream_state = DAC_STREAM_STATE_IDLE;
    return HAL_ERROR;
  } // end if

  HAL_StatusTypeDef status = HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)stream_buffer, DAC_STREAM_BUFFER_SIZE, DAC_ALIGN_12B_R);

  if (status != HAL_OK)
  {
    stream_state = DAC_STREAM_STATE_IDLE;
    return status;
  } // end if status not ok

  status = HAL_TIM_Base_Start(&htim6);

  if (status != HAL_OK)
  {
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    stream_state = DAC_STREAM_STATE_IDLE;
    return status;
  } // end if

  stream_running = 1U;
  return HAL_OK;
} // end function DAC_Stream_StartSequence

void DAC_Stream_Process(void)
{
  if (stream_running == 0U)
  {
    return;
  } // end if

  if (fill_first_half_request != 0U)
  {
    fill_first_half_request = 0U;
    FillStreamBuffer(0U, DAC_STREAM_HALF_SIZE);
  } // end if
  
  if (fill_second_half_request != 0U)
  {
    fill_second_half_request = 0U;
    FillStreamBuffer(DAC_STREAM_HALF_SIZE, DAC_STREAM_HALF_SIZE);
  } // end if
} // end function DAC_Stream_Process

void DAC_Stream_Stop(void)
{
  stream_running = 0U;

  HAL_TIM_Base_Stop(&htim6);
  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);

  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_MID);
  fill_first_half_request = 0U;
  fill_second_half_request = 0U;
  stream_state = DAC_STREAM_STATE_IDLE;
} // end function DAC_Stream_Stop

uint8_t DAC_Stream_IsRunning(void)
{
  return stream_running;
} // end function DAC_Stream_IsRunning

uint8_t DAC_Stream_IsFinished(void)
{
  return stream_finished;
} // end function DAC_Stream_IsFinished

uint8_t DAC_Stream_HasError(void)
{
  return stream_error;
} // end function DAC_Stream_HasError

static void HandleDrainCallback(void)
{
  if (stream_state != DAC_STREAM_STATE_DRAIN)
  {
    return;
  } // end if

  if (drain_callback_count < 0xFFU)
  {
    drain_callback_count++;
  } // end if

  if (drain_callback_count >= DRAIN_CALLBACK_TAEGET)
  {
    stream_state = DAC_STREAM_STATE_FINISHED;
    stream_finished = 1U;
  } // end if
} // end function HandleDrainCallback

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac_handle)
{
  if ((hdac_handle == NULL) || (hdac_handle->Instance != DAC) || (stream_running == 0U))
  {
      return;
  } // end if

  fill_first_half_request = 1U;
  HandleDrainCallback();
} // end function HAL_DAC_ConvHalfCpltCallbackCh1

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac_handle)
{
  if ((hdac_handle == NULL) || (hdac_handle->Instance != DAC) || (stream_running == 0U))
  {
      return;
  } // end if 

  fill_second_half_request = 1U;
  HandleDrainCallback();
} // end function HAL_DAC_ConvCpltCallbackCh1

void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac_handle)
{
  if ((hdac_handle == NULL) || (hdac_handle->Instance != DAC))
  {
      return;
  } // end if

  stream_error = 1U;
  stream_running = 0U;
} // end function HAL_DAC_ErrorCallbackCh1