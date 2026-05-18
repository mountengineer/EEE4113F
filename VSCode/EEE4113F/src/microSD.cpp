#include <microSD.h>

static String getNewFilename() {
    char filename[13];
    for (uint16_t i = 0; i < 9999; i++) {
        sprintf(filename, "DATA%04d.CSV", i);
        if (!SD.exists(filename)) {
            return String(filename);
        }
    }
    return "DATA.CSV";   // fallback
}

void prepSDSPI() {
    digitalWrite(IMU_CS_PIN, HIGH);
    SPI.endTransaction();
    delay(5);
    SD.begin(SD_CS_PIN);
}

bool logIMUData(String filename, float *ax, float *ay, float *az, float *gx, float *gy, float *gz, uint16_t length, uint16_t &total_written) {
    File file = SD.open(filename, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file.");
        return false;
    }
    if (length + total_written > FINAL_SAMPLES) {
        length = FINAL_SAMPLES - total_written;
    }
    length = min(length, FINAL_SAMPLES - total_written);
    for (uint16_t i = 0; i < length; i++) {
        file.write((uint8_t*)&gx[i], 4);
        file.write((uint8_t*)&gy[i], 4);
        file.write((uint8_t*)&gz[i], 4);
        file.write((uint8_t*)&ax[i], 4);
        file.write((uint8_t*)&ay[i], 4);
        file.write((uint8_t*)&az[i], 4);
    }
    total_written += length;

    file.close();
    return true;
}

void printLog(String filename) {
    File file = SD.open(filename, FILE_READ);
    if (!file) {
        Serial.println("Could not read file.");
        return;
    }
    float read;
    while (file.available() >= (int)sizeof(float)) {
        file.read((uint8_t*)&read, sizeof(float));
        Serial.println(read, 6);
    }
}

void readLog(String filename, float *gx, float *gy, float *gz, float *ax, float *ay, float *az) {
    File file = SD.open(filename, FILE_READ);
    if (!file) {
        Serial.println("Could not read file.");
        return;
    }
    for (uint16_t i = 0; i < FINAL_SAMPLES; i++) {
        file.read((uint8_t*)&gx[i], 4);
        file.read((uint8_t*)&gy[i], 4);
        file.read((uint8_t*)&gz[i], 4);
        file.read((uint8_t*)&ax[i], 4);
        file.read((uint8_t*)&ay[i], 4);
        file.read((uint8_t*)&az[i], 4);
    }
}