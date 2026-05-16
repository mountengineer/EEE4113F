#include <DMA.h>

// ZeroDMA objects — one for TX (sending address), one for RX (receiving data)
static Adafruit_ZeroDMA dma_tx;
static Adafruit_ZeroDMA dma_rx;

// DMA descriptors — define the actual transfer parameters
static DmacDescriptor *tx_desc;
static DmacDescriptor *rx_desc;

// TX buffer: register address byte followed by dummy bytes to clock out data
// +1 for the address byte
static uint8_t dma_tx_buf[FIFO_THRESHOLD * 2 + 1];

// RX buffer: first byte is garbage (received during address phase), rest is data
uint8_t dma_rx_buf[FIFO_THRESHOLD * 2 + 1];

static volatile bool dma_done = false;

// Called by ZeroDMA when RX transfer completes
static void dmaCallback(Adafruit_ZeroDMA *dma) {
    // Deassert CS — transfer is complete
    digitalWrite(IMU_CS_PIN, HIGH);
    prevState = currentState;
    currentState = READING_IMU;
    dma_done = true;
}

void initDMA() {
    // Prepare TX buffer — address byte with read bit set, then dummy 0xFF bytes
    // The SAMD51 SPI peripheral needs something in TX to generate clock pulses
    dma_tx_buf[0] = LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L | 0x80;  // read bit
    memset(&dma_tx_buf[1], 0xFF, FIFO_THRESHOLD * 2);           // dummy TX bytes

    // Allocate DMA channels
    // ZeroDMA automatically picks available channels
    if (dma_tx.allocate() != DMA_STATUS_OK) {
        Serial.println("DMA TX alloc failed");
        return;
    }
    if (dma_rx.allocate() != DMA_STATUS_OK) {
        Serial.println("DMA RX alloc failed");
        return;
    }

    // Configure TX job — send from dma_tx_buf to SPI DATA register
    // SPI peripheral on SERCOM1 for ItsyBitsy M4
    tx_desc = dma_tx.addDescriptor(
        dma_tx_buf,                          // source address
        (void *)(&SERCOM1->SPI.DATA.reg),    // destination: SPI data register
        1,                                   // byte count — set dynamically in startDMARead
        DMA_BEAT_SIZE_BYTE,                  // transfer one byte at a time
        true,                                // increment source address
        false                                // do NOT increment destination (always same register)
    );

    // Configure RX job — receive from SPI DATA register into dma_rx_buf
    rx_desc = dma_rx.addDescriptor(
        (void *)(&SERCOM1->SPI.DATA.reg),    // source: SPI data register
        dma_rx_buf,                          // destination address
        1,                                   // byte count — set dynamically
        DMA_BEAT_SIZE_BYTE,
        false,                               // do NOT increment source
        true                                 // increment destination address
    );

    // Trigger TX DMA when SPI TX register is empty (ready for next byte)
    dma_tx.setTrigger(SERCOM1_DMAC_ID_TX);
    dma_tx.setAction(DMA_TRIGGER_ACTON_BEAT);  // trigger one beat per request

    // Trigger RX DMA when SPI RX register has data
    dma_rx.setTrigger(SERCOM1_DMAC_ID_RX);
    dma_rx.setAction(DMA_TRIGGER_ACTON_BEAT);

    // Attach callback to RX channel — fires when all bytes received
    dma_rx.setCallback(dmaCallback);

    Serial.println("DMA initialised");
}

void startDMARead(uint16_t numWords) {
    uint16_t numBytes = numWords * 2 + 1;  // +1 for address byte

    dma_done = false;

    // Update descriptor lengths for this transfer
    tx_desc->BTCNT.reg = numBytes;
    rx_desc->BTCNT.reg = numBytes;

    // Assert CS to begin SPI transaction
    digitalWrite(IMU_CS_PIN, HIGH);   // ensure it starts high
    delayMicroseconds(1);
    digitalWrite(IMU_CS_PIN, LOW);    // assert

    // Start both DMA jobs simultaneously
    // TX must start first to generate clock, RX captures the result
    dma_rx.startJob();
    dma_tx.startJob();
}

bool isDMAComplete() {
    return dma_done;
}

void DMAPrep() {
    dma_done = false;
}

void parseDMABuffer(LSM6DS3 &myIMU, uint16_t numSamples, float *ax, float *ay, float *az, float *gx, float *gy, float *gz, uint16_t &data_index) {

    // Data starts at byte 1 (byte 0 was received during address phase)
    uint8_t *buf = &dma_rx_buf[1];

    for (uint16_t i = 0; i < numSamples; i++) {
        uint16_t offset = i * 12;  // 6 words * 2 bytes each

        // Reconstruct 16-bit signed integers from two bytes (little-endian)
        int16_t raw_gx = (int16_t)(buf[offset+0]  | (buf[offset+1]  << 8));
        int16_t raw_gy = (int16_t)(buf[offset+2]  | (buf[offset+3]  << 8));
        int16_t raw_gz = (int16_t)(buf[offset+4]  | (buf[offset+5]  << 8));
        int16_t raw_ax = (int16_t)(buf[offset+6]  | (buf[offset+7]  << 8));
        int16_t raw_ay = (int16_t)(buf[offset+8]  | (buf[offset+9]  << 8));
        int16_t raw_az = (int16_t)(buf[offset+10] | (buf[offset+11] << 8));

        // calcGyro and calcAccel take int16_t raw values
        gx[data_index + i] = myIMU.calcGyro(raw_gx);
        gy[data_index + i] = myIMU.calcGyro(raw_gy);
        gz[data_index + i] = myIMU.calcGyro(raw_gz);
        ax[data_index + i] = myIMU.calcAccel(raw_ax);
        ay[data_index + i] = myIMU.calcAccel(raw_ay);
        az[data_index + i] = myIMU.calcAccel(raw_az);
    }
    data_index += numSamples;
}