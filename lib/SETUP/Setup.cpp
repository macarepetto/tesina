#include "Setup.h"

Setup::Setup(const char* id)
    : device_id_(id), pcnt_(), pps_(pcnt_), sqw_(pcnt_),
      gps_(Serial2, PIN_GPS_RX, PIN_GPS_TX, PIN_PPS_GPS, GPS_BAUD),
      rtc_(), kalman_(), clock_gnss_(pcnt_),
      clock_rtc_(pcnt_, sqw_, rtc_),
      clock_kalman_(pcnt_, clock_rtc_, kalman_),
      wifi_(WIFI_SSID, WIFI_PASS, SERVER_IP, SERVER_PORT, device_id_),
      telemetry_queue_(wifi_) {}

void Setup::halt(const char* message) {
    Serial.println(message);
    while (true) delay(1000);
}

void Setup::begin() {
    Serial.begin(115200);
    delay(300);
    if (!rtc_.begin()) halt("ERROR: no se detectó DS3231 por I2C");

    int8_t aging_anterior = 0;
    if (!rtc_.readAging(aging_anterior))
        halt("ERROR: no se pudo leer aging");
    if (!rtc_.applyAgingValue(AGING_VALUE))
        halt("ERROR: no se pudo aplicar/verificar aging");
    aging_aplicado_ = rtc_.getAppliedAging();
    gps_.beginNmeaOnly();

    if (!pcnt_.begin(PIN_32K_RTC)) halt("ERROR: PCNT");
    // El 500000 es la barrera ORIGINAL entre flancos PPS; no se
    // valida aquí la duración/los ticks (eso sigue en main_kalman).
    if (!pps_.begin(PIN_PPS_GPS, RISING, 500000UL, 0, INPUT))
        halt("ERROR: captura PPS");
    if (!clock_rtc_.begin(PIN_SQW_RTC))
        Serial.println("ADVERTENCIA: RTC/SQW no disponible");

    if (!telemetry_queue_.begin())
        halt("ERROR: no se pudo crear cola/tarea de telemetría");
    Serial.printf("Aging anterior: %d | aplicado: %d\n",
                  int(aging_anterior), int(aging_aplicado_));
}
