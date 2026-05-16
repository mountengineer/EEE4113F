#include <config.h>
#include <IMU.h>
#include <microSD.h>
#include <filter.h>
#include <process.h>
#include <DMA.h>

// Variable declarations here

LSM6DS3 myIMU(SPI_MODE, IMU_CS_PIN);
uint16_t data_index = 0;

SensorChannel sense_ax = initSensorChannel();
SensorChannel sense_ay = initSensorChannel();
SensorChannel sense_az = initSensorChannel();
SensorChannel sense_gx = initSensorChannel();
SensorChannel sense_gy = initSensorChannel();
SensorChannel sense_gz = initSensorChannel();

// IMU data:
static float ax[(UNLOADS_PER_DEC + 1) * FIFO_THRESHOLD / AXES];
static float ay[(UNLOADS_PER_DEC + 1) * FIFO_THRESHOLD / AXES];
static float az[(UNLOADS_PER_DEC + 1) * FIFO_THRESHOLD / AXES];
static float gx[(UNLOADS_PER_DEC + 1) * FIFO_THRESHOLD / AXES];
static float gy[(UNLOADS_PER_DEC + 1) * FIFO_THRESHOLD / AXES];
static float gz[(UNLOADS_PER_DEC + 1) * FIFO_THRESHOLD / AXES];

static float ax_small[(UNLOADS_PER_DEC + 1) * FIFO_THRESHOLD / (AXES * DEC1 * DEC2)];
static float ay_small[(UNLOADS_PER_DEC + 1) * FIFO_THRESHOLD / (AXES * DEC1 * DEC2)];
static float az_small[(UNLOADS_PER_DEC + 1) * FIFO_THRESHOLD / (AXES * DEC1 * DEC2)];
static float gx_small[(UNLOADS_PER_DEC + 1) * FIFO_THRESHOLD / (AXES * DEC1 * DEC2)];
static float gy_small[(UNLOADS_PER_DEC + 1) * FIFO_THRESHOLD / (AXES * DEC1 * DEC2)];
static float gz_small[(UNLOADS_PER_DEC + 1) * FIFO_THRESHOLD / (AXES * DEC1 * DEC2)];

static float avgPSD[M+1];
static float mm2;
static float mm1;
static float m0;
static float m1;
static float m2;
static float m3;
static float SWH;

arm_rfft_fast_instance_f32 S2;
arm_rfft_fast_instance_f32 S1;


// put function declarations here:
void imuISR();

void setup() {
  // put your setup code here, to run once:
  arm_rfft_fast_init_f32(&S1, PADDED_FOURIER_LEN);
  arm_rfft_fast_init_f32(&S2, 2*M);
  Serial.begin(921600);
  while (!Serial);
  initIMU(myIMU);

  // Starts the Fifo listening
  myIMU.fifoBegin();

  if (SD.begin(11)) {
    Serial.println("SD initialised successfully.");
    SD.rmdir("TEST.TXT");
    SD.remove("TEST.TXT");
  } else {
    Serial.println("SD initialisation failed.");
  }

  // Flush FIFO via register writes
  flushFIFO(myIMU);

  // Sets interrupt pin as input
  pinMode(IMU_INT1_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), imuISR, RISING);
  verifyIMURegisters(myIMU);
}

void loop() {
  // put your main code here, to run repeatedly:
  switch (currentState) {
    case IDLE:
    {

    }
    break;
    case READING_IMU:
    {
      if (readFifo(myIMU, data_index, ax, ay, az, gx, gy, gz)) {
        currentState = FILTERING;
      } else {
        currentState = prevState;
      }
      Serial.print("Gx: "); Serial.println(gx[data_index-1], 6);
      Serial.print("Gy: "); Serial.println(gy[data_index-1], 6);
      Serial.print("Gz: "); Serial.println(gz[data_index-1], 6);
      Serial.print("Ax: "); Serial.println(ax[data_index-1], 6);
      Serial.print("Ay: "); Serial.println(ay[data_index-1], 6);
      Serial.print("Az: "); Serial.println(az[data_index-1], 6);
    }
    break;
    case FILTERING:
    {
      // digitalWrite(IMU_CS_PIN, HIGH);
      // uint16_t numSpikes, numUnresponsive;
      // uint16_t samples = prepForWrite(ax, ax_small, numSpikes, numUnresponsive, data_index, sense_ax);
      // prepForWrite(ay, ay_small, numSpikes, numUnresponsive, data_index, sense_ay);
      // prepForWrite(az, az_small, numSpikes, numUnresponsive, data_index, sense_az);
      // prepForWrite(gx, gx_small, numSpikes, numUnresponsive, data_index, sense_gx);
      // prepForWrite(gy, gy_small, numSpikes, numUnresponsive, data_index, sense_gy);
      // prepForWrite(gz, gz_small, numSpikes, numUnresponsive, data_index, sense_gz);
      // Serial.println("Filtering complete. Writing...");
      // if (logIMUData("TEST.TXT", ax_small, ay_small, az_small, gx_small, gy_small, gz_small, samples)) {
      //   Serial.println("Writing Complete.");
      // }
      // currentState = PROCESSING;
      for (uint16_t i = 0; i < data_index; i++) {
        if (az[i] > -0.7f) {
          Serial.println("Seems something isn't quite working...");
          Serial.println(az[i], 6);
        }
      } 
      data_index = 0;
      if (currentState == FILTERING) { // In case interrupted
        currentState = IDLE;
      }
    }
    break;
    case PROCESSING:
      updateIMU(myIMU, 0, 2);
      //printLog("TEST.TXT");
      fullPipeline(S1, S2, /*gx, gy, gz, ax, ay,*/ az, avgPSD, mm2, mm1, m0, m1, m2, m3, SWH);
      currentState = IDLE;
    break;
    default:

    break;
  }
}

// put function definitions here:
void imuISR() {
  prevState = currentState;
  currentState = READING_IMU;
}