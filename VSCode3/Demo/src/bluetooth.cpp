/**
 * bluetooth.cpp
 * ═══════════════════════════════════════════════════════════════════
 * HC-05 Bluetooth UART implementation — ItsyBitsy M4 (SAMD51)
 * See bluetooth.h for wiring, protocol, and API documentation.
 *
 * Key difference from the ESP8266 version:
 *   SoftwareSerial is replaced by a true hardware UART built on SERCOM4.
 *   A2 = PB08 (SERCOM4 pad 0, RX) and A3 = PB09 (SERCOM4 pad 1, TX).
 *   pinPeripheral() in bluetoothBegin() switches those pins from their
 *   default analogue function to the SERCOM peripheral mux.
 */

#include "bluetooth.h"
//class SAMD_ISR_Timer;             // Forward declaration of the class
//extern SAMD_ISR_Timer myISRTimer; // Points to main.cpp's instance
extern void timerHandler();

// ── Hardware UART definition ──────────────────────────────────────────────────
//
// Uart(SERCOM* sercom, uint8_t pinRX, uint8_t pinTX, SercomRXPad rxPad, SercomUartTXPad txPad)
//
// A2 = PB08 → SERCOM4 pad 0 (RX)
// A3 = PB09 → SERCOM4 pad 1 (TX)
// Both pins use PIO_SERCOM (not ALT) on the ItsyBitsy M4 variant.
//
Uart BT_SERIAL(&sercom4, BT_PIN_RX, BT_PIN_TX, BT_SERCOM_RX_PAD, BT_SERCOM_TX_PAD);

// // ── Required SERCOM IRQ handlers ──────────────────────────────────────────────
// // SAMD51 splits each SERCOM into four sub-interrupts — all four must be
// // forwarded to IrqHandler() or received bytes will be silently dropped.
void SERCOM4_0_Handler() { BT_SERIAL.IrqHandler(); }
void SERCOM4_1_Handler() { BT_SERIAL.IrqHandler(); }
void SERCOM4_2_Handler() { BT_SERIAL.IrqHandler(); }
void SERCOM4_3_Handler() { BT_SERIAL.IrqHandler(); }

// ── Global state ──────────────────────────────────────────────────────────────
BuoyState buoyState;
bool      imuRunning = false;

// ── Module-private state ──────────────────────────────────────────────────────
static unsigned long _lastStatus = 0;
static unsigned long _lastImu    = 0;
static String        _rxBuf      = "";

// ── Command handler ───────────────────────────────────────────────────────────

static void handleCommand(const String& cmd, LSM6DS3 &myIMU) {
    Serial.print("[BT RX] ");
    Serial.println(cmd);
    if (cmd.startsWith("{")) {
        return; 
    }
    else if (cmd == "STATUS") {
        sendStatus();
    }
    else if (cmd.startsWith("SET_MODE:")) {
        int modeNum = cmd.substring(9).toInt();
        switch (modeNum) {
            case 1:
                if (imuRunning) {
                    currentState = TESTING_IMU;
                    prevState = TESTING_IMU;
                    Serial.println("Switching to IMU testing mode.");
                } else {
                    currentState = CONFIG;
                    prevState = CONFIG;
                    Serial.println("Switching to configuration mode.");
                }
            break;
            case 2:

            break;
            case 3:
                buoyState.mode = 3;
                sendAck("SET_MODE", "03");
                Serial.println("[BT] Mode changed to 3");
                BT_SERIAL.end();            // Shut down the SERCOM UART
                pinMode(BT_PIN_TX, OUTPUT);        // Change ItsyBitsy TX pin to standard output
                digitalWrite(BT_PIN_TX, LOW);

                delay(50);

                bluetoothTimer();
                return;
            break;
            default:
                modeNum = -1; // Invalid mode
        }
        if (modeNum >= 1 && modeNum <= 3) {
            buoyState.mode = (uint8_t)modeNum;
            char p[4];
            snprintf(p, sizeof(p), "%02d", modeNum);
            sendAck("SET_MODE", p);
            Serial.print("[BT] Mode changed to ");
            Serial.println(modeNum);
        } else {
            Serial.println("[BT] SET_MODE: invalid mode (must be 1-3)");
        }
    }
    else if (cmd == "GET_LIVE" || cmd == "START_IMU") {
        imuRunning = true;
        currentState = TESTING_IMU;
        Serial.println("Switching to IMU testing mode.");
        updateIMU(myIMU, SAMPLE_RATE_HZ, 3);
        flushFIFO(myIMU);
        _lastImu   = millis();
        Serial.println("[BT] IMU stream started");
    }
    else if (cmd == "STOP") {
        imuRunning = false;
        currentState = CONFIG;
        Serial.println("Switching back to configuration mode.");
        updateIMU(myIMU, 0, 3);
        Serial.println("[BT] IMU stream stopped");
    }
    else if (cmd == "LORA_TEST") {
        // Stub — real LoRa test is triggered here; replace with your LoRa driver call
        // e.g.: bool ack = loraTest(); sendLoraResult(true, ack, buoyState.loraRSSI);
        Serial.println("[BT] LORA_TEST received — wire up your LoRa driver here");
        sendLoraResult(false, false, 0.0f);
    }
    else if (cmd.startsWith("LORA_SEND:")) {
        String content = cmd.substring(10);
        Serial.print("[BT] LORA_SEND: ");
        Serial.println(content);
        // Stub — pass content to your LoRa send function
        // e.g.: bool ack = loraSend(content.c_str()); sendLoraResult(true, ack, buoyState.loraRSSI);
        sendLoraResult(false, false, 0.0f);
    }
    else if (cmd == "GET_SD") {
        // Stub — call your SD/wave-processing module here
        // e.g.: float psd[15]; int n; float Hs, Tz; sdGetSummary(&Hs, &Tz, &n, psd);
        //       sendSDSummary(Hs, Tz, n, psd, 15);
        Serial.println("[BT] GET_SD received — wire up your SD module here");
    }
    else {
        char buf[96];
        snprintf(buf, sizeof(buf),
            "{\"type\":\"error\",\"msg\":\"Unknown cmd: %s\"}\n",
            cmd.c_str());
        BT_SERIAL.print(buf);
        Serial.print("[BT] Unknown command: ");
        Serial.println(cmd);
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

void bluetoothBegin() {
    // Start the SERCOM UART at the configured baud rate
    BT_SERIAL.begin(BT_BAUD);

    // Reassign A2 and A3 from default GPIO/analogue to SERCOM peripheral mux.
    // PIO_SERCOM_ALT selects the "alt" mux column (MUX D on SAMD51) which
    // maps SERCOM0 onto PA06/PA07. Without this the pins stay as GPIO and
    // the UART hardware never sees any signal.
    pinPeripheral(BT_PIN_RX, PIO_SERCOM_ALT);   // PB08 → SERCOM4 primary mux
    pinPeripheral(BT_PIN_TX, PIO_SERCOM_ALT);   // PB09 → SERCOM4 primary mux

    buoyState.startMs = millis();

    while (BT_SERIAL.available()) {
        BT_SERIAL.read(); 
    }
    _rxBuf = "";

    Serial.println("[BT] HC-05 UART ready (SERCOM0)");
    Serial.print("[BT] RX=A2  TX=A3  BAUD=");
    Serial.println(BT_BAUD);
}

void bluetoothUpdate(LSM6DS3 &myIMU) {
    unsigned long now = millis();

    // 1. Periodic STATUS heartbeat ─────────────────────────────────────────────
    if (now - _lastStatus >= BT_STATUS_INTERVAL_MS) {
        _lastStatus = now;
        sendStatus();
    }

    // 2. IMU stream ────────────────────────────────────────────────────────────
    // buoyState.ax/ay/az/gx/gy/gz must be kept current by your IMU module.
    if (imuRunning && hasNewBT) {
        sendImuFrame();
        hasNewBT = false;
        Serial.println("Updated webpage IMU data.");
    }

    // 3. Receive and parse incoming commands ───────────────────────────────────
    while (BT_SERIAL.available()) {
        char c = (char)BT_SERIAL.read();
        Serial.print(c);
        if (c == '\n' || c == '\r') {
            String trimmed = _rxBuf;
            trimmed.trim();
            if (trimmed.length() > 0) {
                handleCommand(trimmed, myIMU);
            }
            _rxBuf = "";
        } else if (_rxBuf.length() < 127) {
            _rxBuf += c;
        }
    }
}

// ── Packet senders ────────────────────────────────────────────────────────────

void sendStatus() {
    uint32_t uptimeSec = (millis() - buoyState.startMs) / 1000UL;

    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"status\","
        "\"node_id\":\"01\","
        "\"mode\":\"%02d\","
        "\"gpsFix\":%s,"
        "\"lat\":%.5f,"
        "\"lon\":%.5f,"
        "\"loraOK\":%s,"
        "\"rssi\":%.1f,"
        "\"uptime\":%lu}\n",
        buoyState.mode,
        buoyState.gpsFix  ? "true" : "false",
        buoyState.lat,
        buoyState.lon,
        buoyState.loraOK  ? "true" : "false",
        buoyState.loraRSSI,
        (unsigned long)uptimeSec
    );

    BT_SERIAL.print(buf);
}

void sendImuFrame() {
    char buf[128];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"imu\","
        "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
        "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f}\n",
        buoyState.ax, buoyState.ay, buoyState.az,
        buoyState.gx, buoyState.gy, buoyState.gz
    );
    BT_SERIAL.print(buf);
    // Not echoed to Serial at 10 Hz — too noisy
}

void sendAck(const char* command, const char* param) {
    char buf[80];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"ack\",\"command\":\"%s\",\"param\":\"%s\"}\n",
        command, param);
    BT_SERIAL.print(buf);
    Serial.print("[BT TX ack] ");
    Serial.print(buf);
}

void sendLoraResult(bool sent, bool ack, float rssi) {
    char buf[100];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"lora_test\","
        "\"sent\":%s,\"ack\":%s,\"rssi\":%.1f}\n",
        sent ? "true" : "false",
        ack  ? "true" : "false",
        rssi
    );
    BT_SERIAL.print(buf);
    Serial.print("[BT TX lora] ");
    Serial.print(buf);
}

void sendSDSummary(float Hs, float Tz, int records, float* psd, int psdLen) {
    // Build the PSD JSON array string: [0.05,0.18,...]
    char psdStr[256] = "[";
    for (int i = 0; i < psdLen; i++) {
        char tmp[12];
        snprintf(tmp, sizeof(tmp), "%.3f", psd[i]);
        strcat(psdStr, tmp);
        if (i < psdLen - 1) strcat(psdStr, ",");
    }
    strcat(psdStr, "]");

    char buf[384];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"sd_summary\","
        "\"Hs\":%.2f,\"Tz\":%.1f,\"records\":%d,"
        "\"psd\":%s}\n",
        Hs, Tz, records, psdStr
    );
    BT_SERIAL.print(buf);
    Serial.print("[BT TX sd] ");
    Serial.print(buf);
}