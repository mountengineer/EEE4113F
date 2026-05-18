#include <config.h>
#include <process.h>
#include <test_data.h>
#include <filter.h>

arm_rfft_fast_instance_f32 S2;
arm_rfft_fast_instance_f32 S1;


void setup() {
    arm_rfft_fast_init_f32(&S1, PADDED_FOURIER_LEN);
    arm_rfft_fast_init_f32(&S2, 2*M);
    Serial.begin(115200);
    float output_buffer[(uint16_t)TEST_DATA_LEN/(DEC1)];
    SensorChannel sensor = initSensorChannel();
    
    // Wait for MATLAB to open the serial port before starting
    while (!Serial) { 
        delay(10); 
    } 
    
    while (Serial.available() == 0) {
        delay(10);
    }

    Serial.read();


    uint16_t numSpikes = 0;
    uint16_t numUnresponsive = 0;

    // Run your math function using the ARM hardware
    // clean(test_input, TEST_DATA_LEN, numSpikes, numUnresponsive, sensor, true);

    // Print the results out over Serial, one per line 
    // for(int i = 0; i < TEST_DATA_LEN; i++) {
    //     // Use 6 decimal places to prevent rounding errors in MATLAB
    //     Serial.println(test_input[i], 6); 
    // }

    // Serial.println(numSpikes);
    // Serial.println(numUnresponsive);
    // numSpikes = 0;
    // numUnresponsive = 0;

    // filtDecimate(test_input, TEST_DATA_LEN, output_buffer, numSpikes, numUnresponsive, sensor);
    /*
    for(int i = 0; i < TEST_DATA_LEN/(DEC1); i++) {
        // Use 6 decimal places to prevent rounding errors in MATLAB
        Serial.print("Index: "); Serial.println(i);
        Serial.println(output_buffer[i], 6); 
    }*/
    float avgPSD[M+1];
    float mm2, mm1, m0, m1, m2, m3, SWH;
    fullPipeline(S1, S2, test_input, avgPSD, mm2, mm1, m0, m1, m2, m3, SWH);
    Serial.println(mm2, 6);
    Serial.println(mm1, 6);
    Serial.println(m0, 6);
    Serial.println(m1, 6);
    Serial.println(m2, 6);
    Serial.println(m3, 6);
    Serial.println(SWH, 6);

    //Serial.println(69);
    //Serial.println(69);
    // Serial.println("Avg PSD:");
    // for (uint16_t i = 0; i < M+1; i++) {
    //     Serial.println(avgPSD[i], 6);
    // }
}

void loop() {
  delay(1000);
}