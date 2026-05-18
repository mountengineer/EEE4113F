#include <process.h>

void fiifif(arm_rfft_fast_instance_f32 &S, float *data, float *out) {
    static float padded_data[PADDED_FOURIER_LEN];
    memset(padded_data, 0, PADDED_FOURIER_LEN * sizeof(float));
    memcpy(padded_data, data, FINAL_SAMPLES * sizeof(float));
    static float complex_fourier[PADDED_FOURIER_LEN];
    // input and output must be the same size (N)
    arm_rfft_fast_f32(&S, padded_data, complex_fourier, 0);
    float bin_width = (float)FINAL_HZ / (float)PADDED_FOURIER_LEN;
    //Serial.println(bin_width, 6);
    complex_fourier[0] = 0.0f;
    complex_fourier[1] = 0.0f;
    // Integrate with Cosine taper
    for (uint16_t i = 1; i < PADDED_FOURIER_LEN/2; i++) {
        float freq = (float)i * bin_width;
        float mult;
        if (freq >= INT_F1 && freq <= INT_F2) {
            mult = 0.5f * (1.0f - cosf(PI * (freq - INT_F1) / (INT_F2 - INT_F1))) * (-1.0f / powf(2.0f * PI * freq, 2.0f));
        } else if (freq > INT_F2 && freq < FINAL_HZ/2) {
            mult = (-1.0f / powf(2.0f * PI * freq, 2.0f));
        } else {
            mult = 0;
        }
        complex_fourier[2*i]   *= mult;
        complex_fourier[2*i+1] *= mult;
    }
    arm_rfft_fast_f32(&S, complex_fourier, padded_data, 1); // inverse fft
    for (uint16_t i = 0; i < M*(K+1); i++) {
        out[i] = padded_data[i+DISCARD_SIZE];
    }
}

void getAvgPSD(arm_rfft_fast_instance_f32 &S1, arm_rfft_fast_instance_f32 &S2, float *data, float *avgPSD) {
    // Serial.println("Pos data:");
    // for (uint16_t i = 0; i < FINAL_SAMPLES; i++) {
    //     Serial.println(data[i], 6);
    // }
    static float pos_data[M*(K+1)];
    fiifif(S1, data, pos_data);
    // for (uint16_t i = 0; i < M*(K+1); i++) {
    //     Serial.println(pos_data[i], 6);
    // }
    static float taper[2*M];
    uint16_t taper_width = floor(2 * M * TAPER_GAMMA / 100);
    float taper_power = 0;
    for (uint16_t i = 0; i < M; i++) {
        if (i < taper_width) {
            taper[i] = 0.5 * (1 - cosf(PI * i / (float)(taper_width)));
        } else {
            taper[i] = 1.0f;
        }
        taper[2 * M - i - 1] = taper[i];
        taper_power += 2 * (taper[i] * taper[i]) / (2 * M);
    }
    static float window[2*M];
    static float fourier[2*M];
    static float denom = 2 * M * FINAL_HZ * taper_power * K;
    memset(avgPSD, 0, (M+1) * sizeof(float));
    for (uint16_t k = 0; k < K; k++) {
        for (uint16_t m = 0; m < 2 * M; m++) {
            window[m] = pos_data[k * M + m] * taper[m];
        }
        arm_rfft_fast_f32(&S2, window, fourier, 0);
        avgPSD[0] = fourier[0] * fourier[0] / (denom * RAO[0] * RAO[0]);
        avgPSD[M] = fourier[1] * fourier[1] / (denom * RAO[M] * RAO[M]);
        for (uint16_t index = 1; index < M; index++) {
            float mag_sq = fourier[2*index] * fourier[2*index] + fourier[2*index+1] * fourier[2*index+1];
            avgPSD[index] += 2.0f * mag_sq / (denom * RAO[index] * RAO[index]);
        }
    }
}   

void getVert(float *gx, float *gy, float *gz, float *ax, float *ay, float *az) {
    Mahony filter;
    filter.begin(SAMPLE_RATE_HZ);
    float roll, pitch;

    for (uint16_t i = 0; i < FINAL_SAMPLES; i++) {
        az[i] *= -1; // Correcting for IMU being upside down
        ay[i] *= -1;
        gz[i] *= -1;
        gy[i] *= -1;
        filter.updateIMU(gx[i], gy[i], gz[i], ax[i], ay[i], az[i]);
        roll = filter.getRollRadians();
        pitch = filter.getPitchRadians();
        // Last term is negative because IMU is upside down
        az[i] = G*(sinf(pitch)*ax[i] + sinf(roll) * cosf(pitch) * ay[i] + cosf(roll) * cosf(pitch) * az[i]); 
    }
}

void getMoments(float *avgPSD, float &mm2, float &mm1, float &m0, float &m1, float &m2, float &m3) {
    float delta_f = ((float)FINAL_HZ/(2.0f*(float)M));
    mm2 = 0;
    mm1 = 0;
    m0 = 0;
    m1 = 0;
    m2 = 0;
    m3 = 0;
    getMoment(avgPSD, mm2, delta_f, -2);
    getMoment(avgPSD, mm1, delta_f, -1);
    getMoment(avgPSD, m0, delta_f, 0);
    getMoment(avgPSD, m1, delta_f, 1);
    getMoment(avgPSD, m2, delta_f, 2);
    getMoment(avgPSD, m3, delta_f, 3);
}

void getMoment(float *avgPSD, float &output, float delta_f, int16_t moment) {
    for (uint16_t i = 1; i < M; i++) { // Ignore the DC and Nyquist bins
        output += powf((i*delta_f), moment) * avgPSD[i];
    }
    output *= delta_f;
}

float significantWH(float m0) {
    return 4 * sqrtf(m0);
}

void fullPipeline(arm_rfft_fast_instance_f32 &S1, arm_rfft_fast_instance_f32 &S2, float *gx, float *gy, float *gz, float *ax, float *ay, float *az, float * avgPSD, float &mm2, float &mm1, float &m0, float &m1, float &m2, float &m3, float &SWH) {
    getVert(gx, gy, gz, ax, ay, az);
    Serial.println("GotVert");
    getAvgPSD(S1, S2, az, avgPSD);
    for (uint16_t i = 0; i < M+1; i++) {
        Serial.println(avgPSD[i], 6);
    }//COMMENT 
    getMoments(avgPSD, mm2, mm1, m0, m1, m2, m3);
    SWH = significantWH(m0);
}