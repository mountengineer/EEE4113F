// GPSModule.h

#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>
#include <TinyGPS++.h>

class GPSModule
{
public:
    GPSModule(HardwareSerial& serialPort);

    void begin(uint32_t baud = 9600);
    void update();

    bool hasFix();
    bool locationUpdated();

    double latitude();
    double longitude();
    double altitudeMeters();
    double speedKmph();

    uint32_t satellites();

    int year();
    int month();
    int day();

    int hourUTC();
    int minuteUTC();
    int secondUTC();

    // South Africa local time (UTC+2)
    int hourLocal();

private:
    HardwareSerial& gpsSerial;
    TinyGPSPlus gps;
};

#endif