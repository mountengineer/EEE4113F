#pragma once
#include <Arduino.h>
#include <Adafruit_ZeroDMA.h>
#include <config.h>
#include <wiring_private.h>   // for SPI peripheral access

// Call once in setup after IMU is initialised
void initDMA();

// Start a DMA FIFO read for numWords 16-bit words
// Returns immediately — check isDMAComplete() before reading buffer
void startDMARead(uint16_t numWords);

// Returns true when the DMA transfer has finished
bool isDMAComplete();

void DMAPrep();

// Raw receive buffer — each word is two bytes, little-endian
// Access after isDMAComplete() returns true
extern uint8_t dma_rx_buf[FIFO_THRESHOLD * 2 + 1];

void parseDMABuffer(LSM6DS3 &myIMU, uint16_t numSamples, float *ax, float *ay, float *az, float *gx, float *gy, float *gz, uint16_t &data_index);