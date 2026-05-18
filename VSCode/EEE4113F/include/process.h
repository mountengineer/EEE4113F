#pragma once

#include <config.h>
#include <MahonyAHRS.h>

void fiifif(arm_rfft_fast_instance_f32 &S, float *data, float *out);

void getAvgPSD(arm_rfft_fast_instance_f32 &S1, arm_rfft_fast_instance_f32 &S2, float *data, float *avgPSD);

void getVert(float *gx, float *gy, float *gz, float *ax, float *ay, float *az);

void getMoments(float *avgPSD, float &mm2, float &mm1, float &m0, float &m1, float &m2, float &m3);

void getMoment(float *avgPSD, float &output, float delta_f, int16_t moment);

float significantWH(float m0);

void fullPipeline(arm_rfft_fast_instance_f32 &S1, arm_rfft_fast_instance_f32 &S2, float *gx, float *gy, float *gz, float *ax, float *ay, float *az, float * avgPSD, float &mm2, float &mm1, float &m0, float &m1, float &m2, float &m3, float &SWH);