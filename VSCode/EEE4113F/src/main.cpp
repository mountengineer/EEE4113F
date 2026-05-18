#include <config.h>
#include <IMU.h>
#include <microSD.h>
#include <filter.h>
#include <process.h>

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
  /*while (Serial.available() == 0) {
      delay(10);
  }

  Serial.read();*/
  initIMU(myIMU);

  // Starts the Fifo listening
  myIMU.fifoBegin();

  if (SD.begin(SD_CS_PIN)) {
    //UNCOMMENT println("SD initialised successfully.");
    SD.rmdir("TEST.TXT");
    SD.remove("TEST.TXT");
  } else {
    //UNCOMMENT Serial.println("SD initialisation failed.");
  }

  // Flush FIFO via register writes
  flushFIFO(myIMU);

  // Sets interrupt pin as input
  pinMode(IMU_INT1_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), imuISR, RISING);
  currentState = IDLE;
  prevState = IDLE;
  // verifyIMURegisters(myIMU);
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
      uint16_t unresponsive_dc = 0; // Don't care
      uint16_t samples = prepForWrite(ax, ax_small, numSpikes, unresponsive_dc, data_index, sense_ax);
      prepForWrite(ay, ay_small, numSpikes, unresponsive_dc, data_index, sense_ay);
      prepForWrite(az, az_small, numSpikes, numUnresponsive, data_index, sense_az);
      prepForWrite(gx, gx_small, numSpikes, unresponsive_dc, data_index, sense_gx);
      prepForWrite(gy, gy_small, numSpikes, unresponsive_dc, data_index, sense_gy);
      prepForWrite(gz, gz_small, numSpikes, unresponsive_dc, data_index, sense_gz);
      data_index = 0; // reset for next round of sampling
      //UNCOMMENT Serial.println("Filtering complete. Writing...");
      prepSDSPI();
      if (logIMUData("TEST.TXT", ax_small, ay_small, az_small, gx_small, gy_small, gz_small, samples, num_written)) {
        //UNCOMMENT Serial.print("Writing Complete. "); Serial.print(num_written); Serial.print("/"); Serial.println(FINAL_SAMPLES);
      }
      
      if (num_written == FINAL_SAMPLES) {
        num_written = 0;
        if (numUnresponsive > FINAL_SAMPLES * DEC1 * DEC2 * UNRESPONSE_TOLERANCE) {
          Serial.println("Too many unresponsive samples");
          Serial.println(numUnresponsive);
          SD.remove("TEST.TXT");
        } else {
          currentState = PROCESSING;
        }
        numUnresponsive = 0;
        numSpikes = 0;
        updateIMU(myIMU, 0 , 1); // stop sampling with IMU
      } else if (num_written > FINAL_SAMPLES) {
        //UNCOMMENT Serial.println("Error: Wrote more samples than expected!");
        exit(1);
      }
      if (currentState == FILTERING) { // In case interrupted
        currentState = IDLE;
      }
    }
    break;
    case PROCESSING:
      //UNCOMMENT Serial.println("Sampling Complete: Processing...");
      memset(gx, 0, sizeof(float) * BUFFER_SIZE);
      memset(gy, 0, sizeof(float) * BUFFER_SIZE);
      memset(gz, 0, sizeof(float) * BUFFER_SIZE);
      memset(ax, 0, sizeof(float) * BUFFER_SIZE);
      memset(ay, 0, sizeof(float) * BUFFER_SIZE);
      memset(az, 0, sizeof(float) * BUFFER_SIZE);
      prepSDSPI();
      readLog("TEST.TXT", gx, gy, gz, ax, ay, az);
      Serial.println("Read completed.");
      fullPipeline(S1, S2, gx, gy, gz, ax, ay, az, avgPSD, mm2, mm1, m0, m1, m2, m3, SWH);
      //UNCOMMENT Serial.println("Processing complete.");
      //UNCOMMENT Serial.print("Significant Wave Height: "); Serial.println(SWH, 6);
      Serial.println(mm2, 6);
      Serial.println(mm1, 6);
      Serial.println(m0, 6);
      Serial.println(m1, 6);
      Serial.println(m2, 6);
      Serial.println(m3, 6);
      Serial.println(SWH, 6);
      currentState = IDLE;
    break;
    case TRANSMITTING:

    break;
    case BEACON:

    break;
    case DUMPING:

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