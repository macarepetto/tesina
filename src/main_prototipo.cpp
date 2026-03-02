#include <Arduino.h>
#include "RTC_Module.h"
#include "GPS_Module.h"

const int PIN_SDA_RTC = 21;
const int PIN_SCL_RTC = 22;
const int PIN_SQW_RTC = 34;

const int GPS_RX = 16;
const int GPS_TX = 17;

RTC_Module rtc(PIN_SDA_RTC, PIN_SCL_RTC, PIN_SQW_RTC);
GPS_Module gps(Serial2, GPS_RX, GPS_TX, 9600);

void setup() {
    Serial.begin(115200);
    delay(800);

    Serial.println("Iniciando RTC + GPS...");

    if (!rtc.begin()) {
        Serial.println("ERROR: No se encuentra el RTC DS3231 :(");
        while (true) delay(1000);
    }

    rtc.adjustToCompileTime();
    gps.begin();

    Serial.println("RTC OK.");
    Serial.println("GPS OK.");
}

void loop() {
    gps.update();

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 1000) {
        lastPrint = millis();

        DateTime nowRtc = rtc.now();
        unsigned long sqw = rtc.getSqwPulses();

        Serial.printf("Hora RTC: %04d/%02d/%02d %02d:%02d:%02d | SQW:%lu | TXT:%lu RMC:%lu GGA:%lu OTR:%lu",
                      nowRtc.year(), nowRtc.month(), nowRtc.day(),
                      nowRtc.hour(), nowRtc.minute(), nowRtc.second(),
                      sqw,
                      gps.getTxtCount(),
                      gps.getRmcCount(),
                      gps.getGgaCount(),
                      gps.getOtherCount());

        if (gps.hasLastValidRmc()) {
            DateTime gpsLocal(2000, 1, 1, 0, 0, 0);
            if (gps.getLastValidLocalDateTime(gpsLocal)) {
                Serial.printf(" | GPS local: %04d/%02d/%02d %02d:%02d:%02d | edad:%lums",
                              gpsLocal.year(), gpsLocal.month(), gpsLocal.day(),
                              gpsLocal.hour(), gpsLocal.minute(), gpsLocal.second(),
                              gps.getLastValidRmcAgeMs());
            }
        } else {
            Serial.print(" | sin RMC valida");
        }

        Serial.println();
    }
}