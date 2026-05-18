#include <IMU.h>
#include <DMA.h>

void initIMU(LSM6DS3 &myIMU) {
    //Over-ride default settings if desired
    myIMU.settings.gyroEnabled = 1;  //Can be 0 or 1
    myIMU.settings.gyroRange = 125;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
    myIMU.settings.gyroSampleRate = SAMPLE_RATE_HZ;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
    myIMU.settings.gyroBandWidth = IMU_BANDWIDTH;  //Hz.  Can be: 50, 100, 200, 400;
    myIMU.settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
    myIMU.settings.gyroFifoDecimation = 1;  //set 1 for on /1

    myIMU.settings.accelEnabled = 1;
    myIMU.settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16
    myIMU.settings.accelSampleRate = SAMPLE_RATE_HZ;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
    myIMU.settings.accelBandWidth = IMU_BANDWIDTH;  //Hz.  Can be: 50, 100, 200, 400;
    myIMU.settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO
    myIMU.settings.accelFifoDecimation = 1;  //set 1 for on /1
    myIMU.settings.tempEnabled = 0;

    //Non-basic mode settings
    myIMU.settings.commMode = 1;

    //FIFO control settings
    myIMU.settings.fifoThreshold = FIFO_THRESHOLD;  //Can be 0 to 4096 (16 bit bytes)
    myIMU.settings.fifoSampleRate = FIFO_SAMPLE_HZ;  //Hz.  Can be: 10, 25, 50, 100, 200, 400, 800, 1600, 3300, 6600
    myIMU.settings.fifoModeWord = 6;  //FIFO mode.
    //FIFO mode.  Can be:
    //  0 (Bypass mode, FIFO off)
    //  1 (Stop when full)
    //  3 (Continuous during trigger)
    //  4 (Bypass until trigger)
    //  6 (Continous mode)
    if (myIMU.begin() != IMU_SUCCESS) {
        //UNCOMMENT Serial.println("IMU init failed");
        while (1);  // TODO: Change error handling for final version
    }

    // Output to INT1 when threshold is exceeded
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x08);
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL1, FIFO_THRESHOLD & 0x00FF);
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL2, (FIFO_THRESHOLD & 0x0F00) >> 8);
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL6_G, 0b00000000);
    //myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, 0b00101110);

    //UNCOMMENT Serial.println("IMU initialised");
    return;
}

void getFifoStatus(LSM6DS3 &myIMU) {
    uint16_t fifoStatus  = myIMU.fifoGetStatus();
    uint16_t fifoCount   = fifoStatus & 0x0FFF;
    bool thresholdFlag   = fifoStatus & 0x8000;   // bit 16
    bool overrunFlag     = fifoStatus & 0x4000;   // bit 15
    bool fullFlag        = fifoStatus & 0x2000;   // bit 14
    bool emptyFlag       = fifoStatus & 0x1000;   // bit 13

    Serial.print("Raw: 0x");        Serial.println(fifoStatus, HEX);
    Serial.print("Word count: ");   Serial.println(fifoCount);
    Serial.print("Threshold: ");    Serial.println(thresholdFlag ? "YES" : "no");
    Serial.print("Overrun: ");      Serial.println(overrunFlag   ? "YES" : "no");
    Serial.print("Full: ");         Serial.println(fullFlag      ? "YES" : "no");
    Serial.print("Empty: ");        Serial.println(emptyFlag     ? "YES" : "no");
}

uint16_t fifoPattern(LSM6DS3 &myIMU) {
    uint8_t patterns[2];
    myIMU.readRegisterRegion(patterns, LSM6DS3_ACC_GYRO_FIFO_STATUS3, 2);
    uint16_t pattern = patterns[0] | ((patterns[1] & 0x03) << 8);
    return pattern;
}

bool readFifo(LSM6DS3 &myIMU, uint16_t &data_index, float* ax, float* ay, float* az, float* gx, float* gy, float* gz)
{
    uint16_t fifoStatus  = myIMU.fifoGetStatus();
    uint16_t fifoCount   = fifoStatus & 0x0FFF;
    // uint16_t pattern = fifoPattern(myIMU);
    uint16_t sample_sets = fifoCount / AXES;
    if (sample_sets > 1) {
        sample_sets -= 2; // Leave the last set for the next read
    }
    // Serial.print("Pattern: 0b"); Serial.println(pattern, BIN);

    for (uint16_t i = data_index; i < sample_sets + data_index; i++) {
        gx[i] = myIMU.calcGyro(myIMU.fifoRead());
        gy[i] = myIMU.calcGyro(myIMU.fifoRead());
        gz[i] = myIMU.calcGyro(myIMU.fifoRead());
        ax[i] = myIMU.calcAccel(myIMU.fifoRead());
        ay[i] = myIMU.calcAccel(myIMU.fifoRead());
        az[i] = myIMU.calcAccel(myIMU.fifoRead());
        uint16_t rem = (myIMU.fifoGetStatus() & 0x0FFF) % AXES;
        if (rem != 0) {
            //UNCOMMENT Serial.println("Found non-zero remainder....");
            for (uint8_t i = 0; i < rem; i++) {
                myIMU.fifoRead(); // discard misaligned words
            }
            // Fill the gap with linear interpolation from last two good samples
            if (data_index >= 2) {
                gx[data_index] = (gx[data_index-1] + gx[data_index-2]) / 2.0f;
                gy[data_index] = (gy[data_index-1] + gy[data_index-2]) / 2.0f;
                gz[data_index] = (gz[data_index-1] + gz[data_index-2]) / 2.0f;
                ax[data_index] = (ax[data_index-1] + ax[data_index-2]) / 2.0f;
                ay[data_index] = (ay[data_index-1] + ay[data_index-2]) / 2.0f;
                az[data_index] = (az[data_index-1] + az[data_index-2]) / 2.0f;
                data_index++;
            }
        }
    }
    data_index += sample_sets;
    // Serial.print("Remainder: "); Serial.println(fifoCount % AXES);
    return data_index >= UNLOADS_PER_DEC * FIFO_THRESHOLD / AXES;
}

/*
bool readFifoDMA(LSM6DS3 &myIMU, uint16_t &data_index, uint16_t numSamples, float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    // Parse the received buffer into float arrays
    parseDMABuffer(myIMU, numSamples, ax, ay, az, gx, gy, gz, data_index);

    // Drain remainder to keep alignment
    // These must be done via normal SPI since they are only a few bytes
    // and DMA overhead would exceed the transfer time
    // for (uint16_t i = 0; i < remainder; i++) {
    //     myIMU.fifoRead();
    // }

    return data_index >= (UNLOADS_PER_DEC * FIFO_THRESHOLD / AXES);
}*/

uint8_t fifoODRBits(uint16_t hz) {
    // Returns the 4-bit ODR field for FIFO_CTRL5 bits [6:3]
    switch(hz) {
        case 10:   return 0b0001;
        case 25:   return 0b0010;
        case 50:   return 0b0011;
        case 100:  return 0b0100;
        case 200:  return 0b0101;
        case 400:  return 0b0110;
        case 800:  return 0b0111;
        case 1600: return 0b1000;
        case 3300: return 0b1001;
        case 6600: return 0b1010;
        default:
            Serial.println("Warning: unsupported FIFO rate, defaulting to 10Hz");
            return 0b0001;
    }
}

void flushFIFO(LSM6DS3 &myIMU) {
    // Bypass mode resets the FIFO hardware
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, 0x00);
    delay(10);

    // Reconstruct FIFO_CTRL5: ODR in bits [6:3], mode in bits [2:0]
    uint8_t odrBits  = fifoODRBits(FIFO_SAMPLE_HZ);
    uint8_t ctrl5    = (odrBits << 3) | 0x06;   // 0x06 = continuous mode
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, ctrl5);

    //UNCOMMENT Serial.print("FIFO restarted, CTRL5=0x");
    //UNCOMMENT Serial.println(ctrl5, HEX);
}

void verifyIMURegisters(LSM6DS3 &myIMU) {
    uint8_t val;

    // FIFO threshold is split across two registers
    // FIFO_CTRL1 holds bits [7:0] of threshold
    // FIFO_CTRL2 holds bits [11:8] of threshold in its lower 4 bits
    
    myIMU.readRegister(&val, LSM6DS3_ACC_GYRO_FIFO_CTRL1);
    Serial.print("FIFO_CTRL1 (threshold low byte): 0x");
    Serial.println(val, HEX);

    myIMU.readRegister(&val, LSM6DS3_ACC_GYRO_FIFO_CTRL2);
    Serial.print("FIFO_CTRL2 (threshold high nibble): 0x");
    Serial.println(val, HEX);

    myIMU.readRegister(&val, LSM6DS3_ACC_GYRO_FIFO_CTRL5);
    Serial.print("FIFO_CTRL5 (mode + ODR): 0x");
    Serial.println(val, BIN);

    myIMU.readRegister(&val, LSM6DS3_ACC_GYRO_INT1_CTRL);
    Serial.print("INT1_CTRL: 0x");
    Serial.println(val, HEX);
}

void updateIMU(LSM6DS3 &myIMU, uint16_t sample_freq_hz, uint16_t full_scale) {
    // sample_freq_hz must be in: 0, 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664
    // full_scale must be in: 0, 1, 2, 3
    // Won't error if not, but won't work as expected
    uint8_t hz = 0;
    bool max_gyro = false;
    switch (sample_freq_hz) {
        case 0: 
            hz = 0;
            break;
        case 13:
            hz = 1;
            break;
        case 26: 
            hz = 2;
            break;
        case 52:
            hz = 3;
            break;
        case 104:
            hz = 4;
            break;
        case 208:
            hz = 5;
            break;
        case 416:
            hz = 6;
            break;
        case 833:
            hz = 7;
            break;
        case 1666:
            hz = 8;
            break;
        case 3332:
            hz = 9;
            max_gyro = true;
            break;
        case 6664:
            hz = 10;
            max_gyro = true;
            break;
        default:
            hz = 0;
            break;
    }

    uint8_t acc_ctrl = ((hz & 0x0F) << 4) | ((full_scale & 0x03) << 2) | 0x03; // 0x03 sets filter to 50Hz
    if (max_gyro) {
        hz = 8;
    }
    uint8_t gyr_ctrl = ((hz & 0x0F) << 4) | ((full_scale & 0x03) << 2);

    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, acc_ctrl);
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, gyr_ctrl);
}