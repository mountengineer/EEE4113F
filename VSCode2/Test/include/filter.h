#pragma once

#include <arm_math.h>
#include <config.h>

struct BiquadState {
    float w1 = 0.0f;
    float w2 = 0.0f;
    bool first = true; // First or second layer
};

struct DecimationChannel {
    BiquadState filter;
    uint16_t counter = 0; 
};

struct FiltDecimator {
    DecimationChannel ch1;
    DecimationChannel ch2;
};

struct SensorChannel {
    FiltDecimator dec;
    float mean = 0;
    float sse = 0;
    uint16_t n = 0; // samples
    bool init = false;
    float s_prev = 0;
};

SensorChannel initSensorChannel();

void getStats(float *data, float32_t &mean, float32_t &std, uint16_t length);

void detrend(float &data, float &s_prev);

bool filtDecimate(float x, float &output, DecimationChannel &channel);

uint16_t filtDecimate(float *input, uint16_t input_length, float* output, uint16_t &numSpikes, uint16_t &numUnresponsive, SensorChannel &sensor);

float biquad(float x, BiquadState &state);

bool clean(float *data, uint16_t length, uint16_t &numSpikes, uint16_t &numUnresponsive, SensorChannel &sensor, bool first_pass);

uint16_t prepForWrite(float *data, float *output, uint16_t &numSpikes, uint16_t &numUnresponsive, uint16_t length, SensorChannel &sensor);