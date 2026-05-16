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
    /*
    while (Serial.available() == 0) {
        delay(10);
    }
    
    // Clear the trigger byte from the buffer
    Serial.read();
    */

    
}

void loop() {
  delay(1000);
}