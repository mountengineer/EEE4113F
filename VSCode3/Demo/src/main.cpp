#include <Arduino.h>
#include <config.h>
#include <IMU.h>
#include <filter.h>
#include <bluetooth.h>
#include <GPSModule.h>
#define USING_TIMER_TC3     true
#include "SAMDTimerInterrupt.h"
#include "SAMD_ISR_Timer.h"

SAMDTimer ITimer(TIMER_TC3);
SAMD_ISR_Timer myISRTimer;

LSM6DS3 myIMU(SPI_MODE, IMU_CS_PIN);
uint16_t data_index = 0;

SensorChannel sense_ax = initSensorChannel();
SensorChannel sense_ay = initSensorChannel();
SensorChannel sense_az = initSensorChannel();
SensorChannel sense_gx = initSensorChannel();
SensorChannel sense_gy = initSensorChannel();
SensorChannel sense_gz = initSensorChannel();

// IMU data:
static float ax[BUFFER_SIZE];
static float ay[BUFFER_SIZE];
static float az[BUFFER_SIZE];
static float gx[BUFFER_SIZE];
static float gy[BUFFER_SIZE];
static float gz[BUFFER_SIZE];
// static float ax[FINAL_SAMPLES];
// static float ay[FINAL_SAMPLES];
// static float az[FINAL_SAMPLES];
// static float gx[FINAL_SAMPLES];
// static float gy[FINAL_SAMPLES];
// static float gz[FINAL_SAMPLES];

static float ax_small[BUFFER_SIZE /(DEC1 * DEC2) + 1];
static float ay_small[BUFFER_SIZE /(DEC1 * DEC2) + 1];
static float az_small[BUFFER_SIZE /(DEC1 * DEC2) + 1];
static float gx_small[BUFFER_SIZE /(DEC1 * DEC2) + 1];
static float gy_small[BUFFER_SIZE /(DEC1 * DEC2) + 1];
static float gz_small[BUFFER_SIZE /(DEC1 * DEC2) + 1];

static float avgPSD[M+1];
static float mm2;
static float mm1;
static float m0;
static float m1;
static float m2;
static float m3;
static float SWH;

static uint16_t num_written = 0;
uint16_t numSpikes = 0;
uint16_t numUnresponsive = 0;

bool first = true;

arm_rfft_fast_instance_f32 S2;
arm_rfft_fast_instance_f32 S1;


#define HW_TIMER_INTERVAL_MS    10L
GPSModule gps(Serial1);

// put function declarations here:
void imuISR();

void timerHandler();

void TimerHandler();

void setup() {
  Serial.begin(921600);
  //while (!Serial) { delay(10); } // wait for Serial to be ready
  if (ITimer.attachInterruptInterval_MS(HW_TIMER_INTERVAL_MS, TimerHandler)) {
    Serial.println("Starting ITimer OK");
  } else {
    Serial.println("Can't set ITimer correctly.");
  }

  // Init SD card before now
  currentState = CONFIG;
  prevState = CONFIG;
}

void loop() {
  switch (currentState) {
    case CONFIG:
      if (first) {
        buoyState.mode = 1;
        digitalWrite(SENSING_PIN, HIGH);
        digitalWrite(BT_PWR_PIN, HIGH);
        digitalWrite(IMU_CS_PIN, LOW);
        digitalWrite(SD_CS_PIN, LOW);
        delay(500);
        initIMU(myIMU);
        myIMU.fifoBegin();
        flushFIFO(myIMU);
        updateIMU(myIMU, 0, 3);
        pinMode(IMU_INT1_PIN, INPUT_PULLDOWN);
        attachInterrupt(IMU_INT1_PIN, imuISR, RISING);
        bluetoothBegin();
        Serial.println("Configuration mode.");
        first = false;
        gps.begin(9600);
      }
      bluetoothUpdate(myIMU);
    break;
    case TESTING_IMU:
    if (first) {
        Serial.println("IMU testing mode.");
        first = false;
      }
      bluetoothUpdate(myIMU);
      if (hasNewBT) {
        hasNewBT = false;
        //Serial.println();
      }
    break;
    case READING_IMU:
      if (readFifo(myIMU, data_index, ax, ay, az, gx, gy, gz)) {
        currentState = FILTERING;
      } else {
        currentState = prevState;
      }
      updateBT(myIMU, buoyState, data_index, gx, gy, gz, ax, ay, az);
    break;
    case FILTERING:
      data_index = 0;
      currentState = prevState;
    break;
    case LOW_POWER:
      
    break;
    case WAKING_UP:
      currentState = CONFIG;
      first = true;
      Serial.println("Woken up.");
    break;
    default:
      currentState = IDLE;
      prevState = IDLE;
    break;
  }
}

// put function definitions here:
void imuISR() {
  prevState = currentState;
  currentState = READING_IMU;
}

void TimerHandler() {
  myISRTimer.run(); // This keeps the software timer pool moving forward!
}

void timerHandler() {
  if (currentState == LOW_POWER) {
    prevState = currentState;
    currentState = WAKING_UP;
  }
}

void GPStoBT() {
  if (gps.locationUpdated()) {
    buoyState.lat = gps.latitude();
    buoyState.lon = gps.longitude();
  }
  buoyState.gpsFix = gps.hasFix();
}

void bluetoothTimer() {
  digitalWrite(BT_PWR_PIN, LOW);
  updateIMU(myIMU, 0, 0);
  digitalWrite(SENSING_PIN, LOW);
  digitalWrite(IMU_CS_PIN, LOW);
  digitalWrite(SD_CS_PIN, LOW);
  myISRTimer.setTimeout(OFF_TIME, timerHandler);
  currentState = LOW_POWER;
}