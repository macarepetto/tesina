#include <Arduino.h>
#include "RTC_Module.h"
#include "GPS_Module.h"
#include "WiFi_Module.h"

// ================== Pines RTC ==================
const int PIN_SDA_RTC = 21;
const int PIN_SCL_RTC = 22;
const int PIN_SQW_RTC = 34;

// ================== Pines GPS ==================
const int GPS_RX  = 16;
const int GPS_TX  = 17;
const int GPS_PPS = 35;

// ================== WiFi / servidor ==================
const char* WIFI_SSID     = "PATAN";
const char* WIFI_PASSWORD = "autoslocos";
const char* SERVER_IP     = "192.168.0.119";   // IP de tu notebook
const uint16_t SERVER_PORT = 8080;
const char* DEVICE_ID = "esp32-prototipo-1";

// ================== Objetos ==================
RTC_Module rtc(PIN_SDA_RTC, PIN_SCL_RTC, PIN_SQW_RTC);
GPS_Module gps(Serial2, GPS_RX, GPS_TX, GPS_PPS, 9600);
WiFi_Module telemetry(WIFI_SSID, WIFI_PASSWORD, SERVER_IP, SERVER_PORT, DEVICE_ID);

void setup() {
    Serial.begin(115200);
    delay(800);

    Serial.println("Iniciando RTC + GPS + PPS + WiFi telemetry...");

    if (!rtc.begin()) {
        Serial.println("ERROR: No se encuentra el RTC DS3231 :(");
        while (true) delay(1000);
    }

    rtc.adjustToCompileTime();
    gps.begin();
    telemetry.begin();

    Serial.println("RTC OK.");
    Serial.println("GPS OK.");
    Serial.println("WiFi telemetry iniciada.");
}

void loop() {
    gps.update();
    telemetry.update();

    static unsigned long lastPrint = 0;
    static unsigned long lastSend  = 0;

    // ===== Log local por Serial =====
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

        if (gps.hasSeenPps()) {
            Serial.printf(" | PPS:%lu (T=%lu us)",
                          gps.getPpsPulseCount(),
                          gps.getPpsPeriodUs());
        } else {
            Serial.print(" | PPS: sin señal");
        }

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

        Serial.printf(" | WiFi:%s | TCP:%s",
                      telemetry.isWifiConnected() ? "OK" : "NO",
                      telemetry.isServerConnected() ? "OK" : "NO");

        Serial.println();
    }

    // ===== Envío al servidor =====
    if (millis() - lastSend >= 1000) {
        lastSend = millis();

        DateTime nowRtc = rtc.now();
        unsigned long sqw = rtc.getSqwPulses();

        String msg = "{";
        msg += "\"id\":\"" + String(DEVICE_ID) + "\",";
        msg += "\"rtc\":\"";
        msg += String(nowRtc.year()) + "-";
        if (nowRtc.month() < 10) msg += "0";
        msg += String(nowRtc.month()) + "-";
        if (nowRtc.day() < 10) msg += "0";
        msg += String(nowRtc.day()) + " ";
        if (nowRtc.hour() < 10) msg += "0";
        msg += String(nowRtc.hour()) + ":";
        if (nowRtc.minute() < 10) msg += "0";
        msg += String(nowRtc.minute()) + ":";
        if (nowRtc.second() < 10) msg += "0";
        msg += String(nowRtc.second()) + "\",";

        msg += "\"sqw\":" + String(sqw) + ",";
        msg += "\"txt\":" + String(gps.getTxtCount()) + ",";
        msg += "\"rmc\":" + String(gps.getRmcCount()) + ",";
        msg += "\"gga\":" + String(gps.getGgaCount()) + ",";
        msg += "\"otr\":" + String(gps.getOtherCount()) + ",";

        if (gps.hasSeenPps()) {
            msg += "\"pps_count\":" + String(gps.getPpsPulseCount()) + ",";
            msg += "\"pps_period_us\":" + String(gps.getPpsPeriodUs()) + ",";
        } else {
            msg += "\"pps_count\":0,";
            msg += "\"pps_period_us\":0,";
        }

        msg += "\"rmc_valid\":";
        msg += gps.hasLastValidRmc() ? "true" : "false";

        if (gps.hasLastValidRmc()) {
            DateTime gpsLocal(2000, 1, 1, 0, 0, 0);
            if (gps.getLastValidLocalDateTime(gpsLocal)) {
                msg += ",\"gps_local\":\"";
                msg += String(gpsLocal.year()) + "-";
                if (gpsLocal.month() < 10) msg += "0";
                msg += String(gpsLocal.month()) + "-";
                if (gpsLocal.day() < 10) msg += "0";
                msg += String(gpsLocal.day()) + " ";
                if (gpsLocal.hour() < 10) msg += "0";
                msg += String(gpsLocal.hour()) + ":";
                if (gpsLocal.minute() < 10) msg += "0";
                msg += String(gpsLocal.minute()) + ":";
                if (gpsLocal.second() < 10) msg += "0";
                msg += String(gpsLocal.second()) + "\"";

                msg += ",\"rmc_age_ms\":" + String(gps.getLastValidRmcAgeMs());
            }
        }

        msg += "}";

        telemetry.sendLine(msg);
    }
}