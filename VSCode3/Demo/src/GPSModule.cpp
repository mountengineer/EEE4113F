// GPSModule.cpp

#include "GPSModule.h"
// G = red, V = black, T = blue, R = green
GPSModule::GPSModule(HardwareSerial& serialPort)
    : gpsSerial(serialPort)
{
}

void GPSModule::begin(uint32_t baud)
{
    gpsSerial.begin(baud);
}

void GPSModule::update()
{
    while (gpsSerial.available())
    {
        gps.encode(gpsSerial.read());
    }
}

bool GPSModule::hasFix()
{
    return gps.location.isValid();
}

bool GPSModule::locationUpdated()
{
    return gps.location.isUpdated();
}

double GPSModule::latitude()
{
    return gps.location.lat();
}

double GPSModule::longitude()
{
    return gps.location.lng();
}

double GPSModule::altitudeMeters()
{
    return gps.altitude.meters();
}

double GPSModule::speedKmph()
{
    return gps.speed.kmph();
}

uint32_t GPSModule::satellites()
{
    return gps.satellites.value();
}

int GPSModule::year()
{
    return gps.date.year();
}

int GPSModule::month()
{
    return gps.date.month();
}

int GPSModule::day()
{
    return gps.date.day();
}

int GPSModule::hourUTC()
{
    return gps.time.hour();
}

int GPSModule::minuteUTC()
{
    return gps.time.minute();
}

int GPSModule::secondUTC()
{
    return gps.time.second();
}

int GPSModule::hourLocal()
{
    return (gps.time.hour() + 2) % 24;
}