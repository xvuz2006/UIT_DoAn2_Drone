#include "fft_telemetry.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern axises gyroAfterNotch;
#define FFT_SAMPLES 2048U
#define LOG_CAPACITY 4096U

typedef struct {
  uint32_t time;
  axises before, notch, after;
} FFT_Sample;

typedef struct {
  uint32_t rpm[4];
  int16_t dshot[4];
  uint8_t enabled;
} FFT_State;

static FFT_State states[FFT_SAMPLES / 32U];
static bool stateSent;

static FFT_Sample samples[FFT_SAMPLES];
static uint8_t logQueue[LOG_CAPACITY];
static char txBuffer[384];
static volatile uint32_t captured, logWrite, logRead, logDropped;
static uint32_t previousCycles, timestampUs, remainingCycles, blockId;
static uint32_t sent, nextCaptureTime;
static uint16_t pendingLogBytes;
static bool transmitting, preferLog, headerSent, endSent;

static uint32_t Text_CRC(const char *data, uint32_t size)
{
  uint32_t crc = 0xFFFFFFFFU;
  for(uint32_t i = 0; i < size; ++i)
  {
    crc ^= (uint8_t)data[i];
    for(uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
  }
  return ~crc;
}

void FFT_Telemetry_Init(void)
{
  huart1.Init.BaudRate = FFT_TELEMETRY_BAUD;
  if(HAL_UART_Init(&huart1) != HAL_OK)
    Error_Handler();
}

void FFT_Telemetry_Log(int ch)
{
  if(logWrite - logRead >= LOG_CAPACITY)
  {
    ++logDropped;
    return;
  }
  logQueue[logWrite % LOG_CAPACITY] = (uint8_t)ch;
  __DMB();
  ++logWrite;
}

void FFT_Telemetry_Capture(const axises *before, const IMU_Data_t *imu,
    const uint32_t rpm[4], const int16_t dshot[4], uint8_t enabled)
{
  uint32_t cycles = DWT->CYCCNT;
  uint32_t elapsed = cycles - previousCycles;
  previousCycles = cycles;
  uint32_t cyclesPerUs = SystemCoreClock / 1000000U;
  timestampUs += elapsed / cyclesPerUs;
  remainingCycles += elapsed % cyclesPerUs;
  timestampUs += remainingCycles / cyclesPerUs;
  remainingCycles %= cyclesPerUs;
  if(captured >= FFT_SAMPLES || before == NULL || imu == NULL || rpm == NULL || dshot == NULL)
    return;
  samples[captured] = (FFT_Sample){timestampUs, *before, gyroAfterNotch, imu->gyro};
  if(captured % 32U == 0)
  {
    FFT_State *state = &states[captured / 32U];
    memcpy(state->rpm, rpm, sizeof(state->rpm));
    memcpy(state->dshot, dshot, sizeof(state->dshot));
    state->enabled = enabled;
  }
  __DMB();
  ++captured;
}

void FFT_Telemetry_Send(void)
{
  if(transmitting)
  {
    if(huart1.gState != HAL_UART_STATE_READY)
      return;
    logRead += pendingLogBytes;
    pendingLogBytes = 0;
    transmitting = false;
  }
  if(endSent && (int32_t)(HAL_GetTick() - nextCaptureTime) >= 0)
  {
    sent = 0;
    headerSent = endSent = stateSent = false;
    ++blockId;
    __DMB();
    captured = 0;
  }
  __DMB();
  bool sendLog = logRead != logWrite && (preferLog || captured < FFT_SAMPLES || endSent);
  uint16_t size = 0;
  if(sendLog)
  {
    uint32_t available = logWrite - logRead;
    pendingLogBytes = available > sizeof(txBuffer) ? sizeof(txBuffer) : (uint16_t)available;
    for(uint16_t i = 0; i < pendingLogBytes; ++i)
      txBuffer[i] = logQueue[(logRead + i) % LOG_CAPACITY];
    for(uint16_t i = pendingLogBytes; i > 0; --i)
    {
      if(txBuffer[i - 1] == '\n')
      {
        pendingLogBytes = i;
        break;
      }
    }
    size = pendingLogBytes;
  }
  else if(captured == FFT_SAMPLES && !endSent)
  {
    if(!headerSent)
      size = snprintf(txBuffer, sizeof(txBuffer), "\r\nFFT_BEGIN,%lu,%u,XYZ9,1000,%g,%g,%g,%u,%u,%u,%u,%u\r\n",
          (unsigned long)blockId, (unsigned int)FFT_SAMPLES, (double)GYRO_CUTOFF,
          (double)ACCEL_CUTOFF, (double)RPM_Q_FACTOR, RPM_MAX_HARMONICS,
          MOTOR_POLES_NUMBER, RPM_MIN_FREQUENCY_HZ, RPM_FADE_RANGE_HZ,
#ifdef USE_RPM_FILTER
          1U);
#else
          0U);
#endif
    else if(sent < FFT_SAMPLES)
    {
      FFT_Sample *sample = &samples[sent];
      int length;
      if(sent % 32U == 0 && !stateSent)
      {
        FFT_State *state = &states[sent / 32U];
        length = snprintf(txBuffer, sizeof(txBuffer), "FFT_STATE,%lu,%lu,%lu,%lu,%lu,%lu,%d,%d,%d,%d,%u",
            (unsigned long)blockId, (unsigned long)sent,
            (unsigned long)state->rpm[0], (unsigned long)state->rpm[1],
            (unsigned long)state->rpm[2], (unsigned long)state->rpm[3],
            state->dshot[0], state->dshot[1], state->dshot[2], state->dshot[3], state->enabled);
      }
      else
        length = snprintf(txBuffer, sizeof(txBuffer), "FFT_SAMPLE,%lu,%lu,%lu,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
            (unsigned long)blockId, (unsigned long)sent, (unsigned long)sample->time,
            (double)sample->before.x, (double)sample->before.y, (double)sample->before.z,
            (double)sample->notch.x, (double)sample->notch.y, (double)sample->notch.z,
            (double)sample->after.x, (double)sample->after.y, (double)sample->after.z);
      if(length < 0 || (size_t)length + 12U >= sizeof(txBuffer))
        return;
      size = length + snprintf(txBuffer + length, sizeof(txBuffer) - length, ",%08lX\r\n",
          (unsigned long)Text_CRC(txBuffer, length));
    }
    else
      size = snprintf(txBuffer, sizeof(txBuffer), "FFT_END,%lu,%lu\r\n", (unsigned long)blockId, (unsigned long)logDropped);
  }
  if(size == 0)
    return;
  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t *)txBuffer, size) == HAL_OK)
  {
    transmitting = true;
    preferLog = !sendLog || txBuffer[size - 1] != '\n';
    if(!sendLog)
    {
      if(!headerSent)
        headerSent = true;
      else if(sent < FFT_SAMPLES)
      {
        if(sent % 32U == 0 && !stateSent)
          stateSent = true;
        else
        {
          ++sent;
          stateSent = false;
        }
      }
      else
      {
        endSent = true;
        nextCaptureTime = HAL_GetTick() + 5000U;
      }
    }
  }
  else
    pendingLogBytes = 0;
}
