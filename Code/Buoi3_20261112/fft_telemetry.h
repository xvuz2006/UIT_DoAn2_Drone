#ifndef FFT_TELEMETRY_H
#define FFT_TELEMETRY_H

#include "main.h"
#include "icm20948.h"

#define FFT_TELEMETRY_ENABLED 1
#define FFT_TELEMETRY_BAUD 115200U

void FFT_Telemetry_Init(void);
void FFT_Telemetry_Capture(const axises *before, const IMU_Data_t *imu,
    const uint32_t rpm[4], const int16_t dshot[4], uint8_t enabled);
void FFT_Telemetry_Log(int ch);
void FFT_Telemetry_Send(void);

#endif
