#pragma once
/**
 * bluetooth.h
 * ═══════════════════════════════════════════════════════════════════
 * HC-05 Bluetooth UART interface — ItsyBitsy M4 (SAMD51)
 *
 * Uses a hardware SERCOM UART (SERCOM3) instead of SoftwareSerial,
 * which is not supported on SAMD51. Hardware UART gives reliable
 * comms with zero CPU bit-banging overhead.
 *
 * Wiring:
 *   HC-05 TX → A2  (ItsyBitsy M4 RX — SERCOM3 pad 2)
 *   HC-05 RX → A3  (ItsyBitsy M4 TX — SERCOM3 pad 0)
 *   HC-05 VCC → 3.3 V
 *   HC-05 GND → GND
 *
 *   NOTE: The HC-05 is a 3.3 V logic device — safe to connect directly
 *   to the ItsyBitsy M4's 3.3 V GPIO pins with no level shifter.
 *
 * PlatformIO (platformio.ini):
 *   [env:adafruit_itsybitsy_m4]
 *   platform  = atmelsam
 *   board     = adafruit_itsybitsy_m4
 *   framework = arduino
 *   monitor_speed = 115200
 *
 * Arduino IDE:
 *   Board → Adafruit ItsyBitsy M4 Express  (Adafruit SAMD package)
 *
 * JSON protocol (all newline-terminated, M4 → dashboard):
 *   {"type":"status",   "node_id":"01","mode":"02","gpsFix":true,
 *    "lat":-62.48320,"lon":-58.30210,"loraOK":true,
 *    "rssi":-112.3,"uptime":3600}
 *
 *   {"type":"imu",      "ax":0.123,"ay":-0.045,"az":9.812,
 *    "gx":0.012,"gy":-0.008,"gz":0.003}
 *
 *   {"type":"ack",      "command":"SET_MODE","param":"02"}
 *
 *   {"type":"lora_test","sent":true,"ack":true,"rssi":-115.2}
 *
 *   {"type":"sd_summary","Hs":2.84,"Tz":8.2,"records":144,
 *    "psd":[...N floats...]}
 */

#include <Arduino.h>
#include <wiring_private.h>   // pinPeripheral() — reassigns pins to SERCOM mux

// ── Hardware UART instance ───────────────────────────────────────────────────
// Defined once in bluetooth.cpp; declared extern here so other modules that
// include bluetooth.h can call BT_SERIAL.print() directly if needed.
extern Uart BT_SERIAL;

// ── Pin & SERCOM configuration ───────────────────────────────────────────────
//
// ItsyBitsy M4 / SAMD51 SERCOM assignment:
//
//   Pin  Arduino  Port   SERCOM-alt  Pad   Role
//   A2   A2       PA06   SERCOM0     2     ← RX (HC-05 TX connects here)
//   A3   A3       PA07   SERCOM0     3     → TX (HC-05 RX connects here)
//
//   The "alt" SERCOM mux (PIO_SERCOM_ALT) on PA06/PA07 is SERCOM0.
//   SERCOM3 is used for SPI on the ItsyBitsy M4, so we use SERCOM0 here.
//   If you have a custom board variant, verify against your variant.h.
//
static const uint8_t  BT_PIN_RX        = A3;   // PB08 — SERCOM4 pad 0 ← HC-05 TX
static const uint8_t  BT_PIN_TX        = A2;   // PB09 — SERCOM4 pad 1 → HC-05 RX

// SERCOM4 pad assignments for A2 (PB08) and A3 (PB09):
//   RX = pad 0  →  SERCOM_RX_PAD_0
//   TX = pad 1  →  UART_TX_PAD_0
//
// NOTE: PB08/PB09 use PIO_SERCOM (not PIO_SERCOM_ALT) — the primary mux
// column maps SERCOM4 onto these pins on the ItsyBitsy M4 variant.
static const SercomRXPad     BT_SERCOM_RX_PAD = SERCOM_RX_PAD_1;
static const SercomUartTXPad BT_SERCOM_TX_PAD = UART_TX_PAD_0;

static const long BT_BAUD = 9600;

// ── Timing ───────────────────────────────────────────────────────────────────
static const unsigned long BT_STATUS_INTERVAL_MS = 3000;   // STATUS heartbeat
static const unsigned long BT_IMU_INTERVAL_MS    = 100;    // 10 Hz IMU stream

// ── Shared buoy state ────────────────────────────────────────────────────────
// Defined in bluetooth.cpp. Include this header from any module that needs
// to read or write buoy state (GPS, IMU, LoRa, SD processing, etc.).
struct BuoyState {
    uint8_t  mode     = 1;       // 1=Calibration  2=Active  3=LowPower
    float    lat      = 0.0f;
    float    lon      = 0.0f;
    bool     gpsFix   = false;
    bool     loraOK   = false;
    float    loraRSSI = 0.0f;
    uint32_t startMs  = 0;       // millis() at boot — used for uptime

    // Real sensor fields — written by your sensor modules, read by send functions
    float    Hs  = 0.0f;         // significant wave height (m)
    float    Tz  = 0.0f;         // mean zero-crossing period (s)
    float    ax  = 0.0f;         // accelerometer (m/s²)
    float    ay  = 0.0f;
    float    az  = 0.0f;
    float    gx  = 0.0f;         // gyroscope (°/s)
    float    gy  = 0.0f;
    float    gz  = 0.0f;
};

extern BuoyState buoyState;

// ── IMU stream flag ──────────────────────────────────────────────────────────
// Set true by GET_LIVE / START_IMU command; cleared by STOP.
extern bool imuRunning;

// ── Public API ───────────────────────────────────────────────────────────────

/**
 * bluetoothBegin()
 * Call once from setup() after Serial.begin().
 * Initialises SERCOM0 as a UART and reassigns A2/A3 to the SERCOM peripheral
 * mux so they carry RX and TX instead of acting as analogue inputs.
 */
void bluetoothBegin();
#include <IMU.h> 
/**
 * bluetoothUpdate()
 * Call every loop() iteration — no blocking delay() inside.
 * Handles:
 *   • Periodic STATUS heartbeat every BT_STATUS_INTERVAL_MS
 *   • IMU frame streaming at BT_IMU_INTERVAL_MS when imuRunning == true
 *   • Incoming command parsing from the HC-05
 */
void bluetoothUpdate(LSM6DS3 &myIMU);

// ── Packet senders ───────────────────────────────────────────────────────────

/** Broadcast a full STATUS JSON packet. Reads from buoyState. */
void sendStatus();

/**
 * Send one IMU JSON frame.
 * Reads buoyState.ax/ay/az/gx/gy/gz — update these from your IMU driver
 * each loop() before bluetoothUpdate() is called.
 */
void sendImuFrame();

/**
 * Send an ACK packet confirming a received command.
 * @param command  Command name, e.g. "SET_MODE"
 * @param param    Parameter string, e.g. "02"
 */
void sendAck(const char* command, const char* param);

/**
 * Send a LoRa test-result packet.
 * @param sent  true if the radio transmitted successfully
 * @param ack   true if a remote acknowledgement was received
 * @param rssi  Link RSSI in dBm (negative value)
 */
void sendLoraResult(bool sent, bool ack, float rssi);

/**
 * Send a wave-data SD summary with a full PSD array.
 * Call this from your SD / wave-processing module after computing wave stats.
 * @param Hs      Significant wave height (m)
 * @param Tz      Mean zero-crossing period (s)
 * @param records Number of IMU records processed
 * @param psd     Float array of PSD frequency bins
 * @param psdLen  Number of elements in psd[]
 */
void sendSDSummary(float Hs, float Tz, int records, float* psd, int psdLen);