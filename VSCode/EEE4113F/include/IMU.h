#pragma once

#include <SparkFunLSM6DS3.h>
#include <config.h>

void initIMU(LSM6DS3 &myIMU);

void getFifoStatus(LSM6DS3 &myIMU);

uint16_t fifoPattern(LSM6DS3 &myIMU);

bool readFifo(LSM6DS3 &myIMU, uint16_t &data_index, float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

bool readFifoDMA(LSM6DS3 &myIMU, uint16_t &data_index, uint16_t numSamples, float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

uint8_t fifoODRBits(uint16_t hz);

void flushFIFO(LSM6DS3 &myIMU);

void verifyIMURegisters(LSM6DS3 &myIMU);

void updateIMU(LSM6DS3 &myIMU, uint16_t sample_freq_hz, uint16_t full_scale);