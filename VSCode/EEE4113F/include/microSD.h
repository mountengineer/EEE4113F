#pragma once

#include <Sd.h>
#include <config.h>
#include <Arduino.h>

bool logIMUData(String filename, float *ax, float *ay, float *az, float *gx, float *gy, float *gz, uint16_t length, uint16_t &total_written);
void printLog(String filename);
void prepSDSPI();
void readLog(String filename, float *gx, float *gy, float *gz, float *ax, float *ay, float *az);