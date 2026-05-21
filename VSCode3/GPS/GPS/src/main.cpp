#include <Arduino.h>
#include "GPSModule.h"

// Use Serial1 on the SAMD51
GPSModule gps(Serial1);

void setup()
{
    Serial.begin(115200);

    while (!Serial)
    {
        delay(10);
    }

    Serial.println("Starting GPS...");

    // GPS default baud
    gps.begin(9600);
}

void loop()
{
    gps.update();

    if (gps.locationUpdated())
    {
        Serial.println("===== GPS FIX =====");

        Serial.print("Latitude: ");
        Serial.println(gps.latitude(), 6);

        Serial.print("Longitude: ");
        Serial.println(gps.longitude(), 6);

        Serial.print("Altitude (m): ");
        Serial.println(gps.altitudeMeters());

        Serial.print("Speed (km/h): ");
        Serial.println(gps.speedKmph());

        Serial.print("Satellites: ");
        Serial.println(gps.satellites());

        Serial.print("Date: ");
        Serial.print(gps.day());
        Serial.print("/");
        Serial.print(gps.month());
        Serial.print("/");
        Serial.println(gps.year());

        Serial.print("UTC Time: ");
        Serial.print(gps.hourUTC());
        Serial.print(":");
        Serial.print(gps.minuteUTC());
        Serial.print(":");
        Serial.println(gps.secondUTC());

        Serial.print("Cape Town Time: ");
        Serial.print(gps.hourLocal());
        Serial.print(":");
        Serial.print(gps.minuteUTC());
        Serial.print(":");
        Serial.println(gps.secondUTC());

        Serial.println("===================");
    }
}