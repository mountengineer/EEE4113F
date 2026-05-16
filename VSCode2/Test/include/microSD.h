#pragma once

#include <Sd.h>
#include <config.h>
#include <Arduino.h>

bool logIMUData(String filename, float *ax, float *ay, float *az, float *gx, float *gy, float *gz, uint16_t length);
void printLog(String filename);