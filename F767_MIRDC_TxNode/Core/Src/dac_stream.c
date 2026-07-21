#include "dac_stream.h"
#include "dbpsk_mod.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define PI 3.14159265358979323846f

#define SYNC_CHIRP_REPEAT_COUNT     3U

#define SILENCE_DURATION_SEC        1.0f
#define PAIR_GUARD_DURATION_MS      100.0f

#define CHIRP_DURATION_SEC          0.0035f
#define GI_DURATION_SEC             0.0005f

#define CHIRP_SAMPLES         ((uint32_t)(DAC_FS * CHIRP_DURATION_SEC + 0.5f))
#define GI_SAMPLES            ((uint32_t)(DAC_FS * GI_DURATION_SEC + 0.5f))
#define PAIR_GUARD_SAMPLES    ((uint32_t)(DAC_FS * PAIR_GUARD_DURATION_MS / 1000.0f + 0.5f))
#define SILENCE_SAMPLES       ((uint32_t)(DAC_FS * SILENCE_DURATION_SEC) + 0.5f)

extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim6;

// DMA buffer
#if defined(__GNUC__)
__attribute__((aligned(32)))
#endif
static uint16_t stream_buffer[DAC_STREAM_BUFFER_SIZE];

static volatile uint8_t fill_first_half_request = 0U;
static volatile uint8_t fill_second_half_request = 0U;

static volatile uint8_t stream_running = 0U;
static volatile uint8_t stream_finished = 0U;
static volatile uint8_t stream_error = 0U;

// store the re-built packets
static uint8_t stream_packets[DAC_STREAM_PAIR_COUNT][DAC_STREAM_MAX_PACKET_SIZE];
static uint16_t stream_packet_lengths[DAC_STREAM_PAIR_COUNT];

// status of stream
typedef enum
{
  DAC_STREAM_STATE_IDLE = 0,
  DAC_STREAM_STATE_SYNC_CHIRP,
  DAC_STREAM_STATE_SILENCE,
  DAC_STREAM_STATE_PACKET,
  DAC_STREAM_STATE_PAIR_GUARD,
  DAC_STREAM_STATE_FINISHED
} DAC_StreamState_t;

static DAC_StreamState_t stream_state = DAC_STREAM_STATE_IDLE;

static uint32_t state_sample_index = 0U;

static uint8_t sync_chirp_count = 0U;

/*
Current Frequency Pair index:
  0 -> Pair 1
  1 -> Pair 2
  2 -> Pair 3
  3 -> Pair 4
*/
static uint8_t current_pair_index = 0U;

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
    {28200, 28600, 29000, 29400},
    {29800, 30200, 30600, 31000},
    {31400, 31800, 32200, 32600},
    {33000, 33400, 33800, 34200}
};

static uint16_t FloatToDAC(float singal)
{
  int32_t dac_value = DAC_MID + (int32_t)(DAC_AMP * singal);

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

/* Sync chirp sample */
static uint32_t GetSyncChirpSamples(void)
{
  return (uint32_t)(DAC_FS * SYNC_CHIRP_DURATION + 0.5f);
} // end function GetSyncChirpSamples

static uint16_t GenerateSyncChirpSample(uint32_t n)
{
  const uint32_t total_samples = GetSyncChirpSamples();

  if (total_samples < 2U || n >= total_samples)
  {
    return DAC_MID;
  } // end if sample too small or reach the end

  const float t = (float)n / DAC_FS;

  const float chirp_rate = (SYNC_CHIRP_END_FREQ - SYNC_CHIRP_START_FREQ) / SYNC_CHIRP_DURATION;

  const float phase = 2.0f * PI * (SYNC_CHIRP_START_FREQ * t + 0.5f * chirp_rate * t * t);

  const float window = 0.5f * (1.0f - cosf(2.0f * PI * (float)n / (float)(total_samples - 1U)));

  const float signal = sinf(phase) * window;

  return FloatToDAC(signal);
} // end function GenerateSyncChirpSample

/* Packet chirp symbol sample */
static uint16_t GeneratePacketSample(uint8_t pair_index, uint32_t packet_sample)
{
  const uint16_t packet_len = stream_packet_lengths[pair_index];
  const uint32_t total_symbols = (uint32_t)packet_len * 4U;
  const uint32_t total_packet_samples = total_symbols * SAMPLES_PER_SYMBOL;

  if (packet_sample >= total_packet_samples)
  {
    return DAC_MID;
  } // end if packet_sample too big

  const uint32_t symbol_number = packet_sample / SAMPLES_PER_SYMBOL;
  const uint32_t sample_in_symbol = packet_sample % SAMPLES_PER_SYMBOL;

  const uint32_t byte_index = symbol_number / 4U;
  const uint32_t symbol_in_byte = symbol_number % 4U;
  const uint8_t byte = stream_packets[pair_index][byte_index];

  /*
     * symbol_in_byte：
     *
     * 0 → bit 7:6 → shift 6
     * 1 → bit 5:4 → shift 4
     * 2 → bit 3:2 → shift 2
     * 3 → bit 1:0 → shift 0
  */
  const uint8_t shift = (uint8_t)(6U - 2U * symbol_in_byte);
  const uint8_t symbol = (uint8_t)((byte>>shift) & 0x03U);

  /* output DAC_MID during GI */
  if (sample_in_symbol > CHIRP_SAMPLES)
  {
    return DAC_MID;
  } // end if

  const float center_freq = chirp_center_freq_table[pair_index][symbol];
  const float f_start = center_freq - 100.0f;
  const float f_end = center_freq + 100.0f;
  const float chirp_time = (float)CHIRP_SAMPLES / DAC_FS;
  const float chirp_rate = (f_end - f_start) / chirp_time;
  const float t = (float)sample_in_symbol / DAC_FS;
  const float phase = 2.0 * PI * (f_start * t + 0.5f * chirp_rate * t * t);

  const float window = 0.5f * (1.0f * cosf(2.0f * PI * (float)sample_in_symbol / (float)(CHIRP_SAMPLES - 1U)));
  const float signal = sinf(phase) * window;

  return FloatToDAC(signal);
} // end function GeneratePacketSample

/* State transition */
static uint32_t GetCurrentPacketTotalSamples(void)
{
  const uint32_t total_symbols = (uint32_t)stream_packet_lengths[current_pair_index] * 4U;

  return total_symbols * SAMPLES_PER_SYMBOL;
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
          stream_state = DAC_STREAM_STATE_SILENCE;
        } // end if
      } // end if

      break;
    } // end case DAC_STREAM_STATE_SYNC_CHIRP
  
    case DAC_STREAM_STATE_SILENCE:
    {
      if (state_sample_index >= SILENCE_SAMPLES)
      {
        state_sample_index - 0U;
        current_pair_index = 0U;

        stream_state = DAC_STREAM_STATE_PACKET;
      } // end if

      break;
    } // end case DAC_STREAM_STATE_SILENCE

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
          stream_state = DAC_STREAM_STATE_FINISHED;
        } // end else
      } // end if

      break;
    } // end case DAC_STRAM_STATE_PACKET

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
      break;
  } // end switch
} // end function AdvanceStateIfNeeded

/* Produce next sample */
static uint16_t GenerateNextSample(void)
{
  uint16_t sample = DAC_MID;

  AdvanceStateIfNeeded();

  switch (stream_state)
  {
    case DAC_STREAM_STATE_SYNC_CHIRP:
    {
      sample = GenerateSyncChirpSample(state_sample_index);
      state_sample_index++;

      break;
    } // end case DAC_STREAM_STATE_SYNC_CHIRP
  
    case DAC_STREAM_STATE_SILENCE:
    {
      sample = DAC_MID;
      state_sample_index++;

      break;
    } // end case DAC_STREAM_STATE_SILENCE

    case DAC_STREAM_STATE_PACKET:
    {
      sample = GeneratePacketSample(current_pair_index, state_sample_index);
      state_sample_index++;

      break;
    } // end case DAC_STREAM_STATE_PACKET

    case DAC_STREAM_STATE_PAIR_GUARD:
    {
      sample = DAC_MID;
      state_sample_index++;

      break;
    } // end case DAC_STREAM_STATE_PAIR_GUARD
  
    case DAC_STREAM_STATE_FINISHED:
    {
      sample = DAC_MID;

      stream_finished = 1U;

      break;
    } // end case DAC_STREAM_STATE_FINISHED

    default:
    {
      sample = DAC_MID;
      break;
    } // end case default
  } // end switch

  return sample;
} // end function GenerateNextSample

/* Fill DMA half buffer */
static void FillStreamBuffer(uint32_t offset, uint32_t length)
{
  if ((offset + length) > DAC_STREAM_BUFFER_SIZE)
  {
    stream_error = 1U;
    return;
  } // end if offser + length too big

  for (uint32_t i = 0U; i < length; i++)
  {
    stream_buffer[offset + i] = GenerateNextSample();
  } // end for
} // end function FillStreamBuffer

/* Start / Process / Stop */
HAL_StatusTypeDef DAC_Stream_StartSequence(const uint8_t packets[DAC_STREAM_PAIR_COUNT][DAC_STREAM_MAX_PACKET_SIZE], const uint16_t packet_lengths[DAC_STREAM_PAIR_COUNT])
{
  if (packets == NULL || packet_lengths == NULL)
  {
    return HAL_ERROR;
  } // end if packet and its length is NULL

  if (stream_running != 0U)
  {
    return HAL_BUSY;
  } // end if stream is running

  for (uint8_t pair = 0U; pair < DAC_STREAM_PAIR_COUNT; pair++)
  {
    if (packet_lengths[pair] == 0U || packet_lengths[pair] > DAC_STREAM_MAX_PACKET_SIZE)
    {
      return HAL_ERROR;
    } // end if

    stream_packet_lengths[pair] = packet_lengths[pair];
    memcpy(stream_packets[pair], packets[pair], packet_lengths[pairs]);
  } // end for

  fill_first_half_request = 0U;
  fill_second_half_request = 0U;

  stream_error = 0U;
  stream_finished = 0U;
  stream_running = 0U;

  sync_chirp_count = 0U;
  current_pair_index = 0U;
  state_sample_index = 0U;

  stream_state = DAC_STREAM_STATE_SYNC_CHIRP;

  FillStreamBuffer(0U, DAC_STREAM_HALF_SIZE);
  FillStreamBuffer(DAC_STREAM_HALF_SIZE , DAC_STREAM_HALF_SIZE);

  if (stream_error != 0U)
  {
    return HAL_ERROR
  } // end if error

  HAL_StatusTypeDef status = HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)stream_buffer, DAC_STREAM_BUFFER_SIZE, DAC_ALIGN_12B_R);

  if (status != HAL_OK)
  {
    return status;
  } // end if start dma OK

  status = HAL_TIM_Base_Start(&htim6);

  if (status != HAL_OK)
  {
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);

    return status;
  } // end if TIM start not OK

  stream_running = 1U;

  return HAL_OK;
} // end function DAC_Stream_StartSequence

void DAC_Stream_Process(void)
{
  if (stream_running == 0U)
  {
    return;
  } // end if not running

  if (fill_first_half_request != 0U)
  {
    fill_first_half_request = 0U;
    FillStreamBuffer(0U, DAC_STREAM_HALF_SIZE);
  } // end if first half not request

  if (fill_second_half_request != 0U)
  {
    fill_second_half_request = 0U;
    FillStreamBuffer(DAC_STREAM_BUFFER_SIZE , DAC_STREAM_BUFFER_SIZE);
  } // end if second half not request
} // end DAC_Stream_Process


void DAC_Stream_Stop(void)
{
  stream_running = 0U;

  HAL_TIM_Base_Stop(&htim6);

  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);

  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_MID);
  
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
} // end function DAC_StreamHasError

/* HAL callbacks */
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac_handle)
{
  if ((hdac_handle != NULL) && (hdac_handle->Instance == DAC) && (stream_running != 0U))
  {
    fill_first_half_request = 1U;
  } // end if 
} // end function HAL_DAC_ConvHalfCpltCallbackCh1

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac_handle)
{
  if ((hdac_handle != NULL) && (hdac_handle->Instance == DAC) && (stream_running != 0U))
  {
    fill_second_half_request = 1U;
  } // end if
} // end function HAL_DAC_ConvCpltCallbackCh1

void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac_handle)
{
  if ((hdac_handle != NULL) && (hdac_handle->Instance == DAC))
  {
    stream_error = 1U;
    stream_running = 0U;
  } // end if
} // end function HAL_DAC_ErrorCallbackCh1