#include "Setup.h"

Setup::Setup(const char* device_id)
    : device_id_(device_id),
      pcnt_(),
      pps_(pcnt_),
      sqw_(pcnt_),
      gps_(Serial2, PIN_GPS_RX, PIN_GPS_TX, PIN_PPS_GPS, GPS_BAUD),
      rtc_(),
      kalman_(),
      clock_gnss_(pcnt_, pps_, gps_),
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

    if (!rtc_.begin()) halt("ERROR: DS3231 por I2C");

    int8_t aging_anterior = 0;
    if (!rtc_.readAging(aging_anterior))
        halt("ERROR: no se pudo leer aging");
    if (!rtc_.applyAgingValue(AGING_VALUE))
        halt("ERROR: no se pudo aplicar aging");
    aging_aplicado_ = rtc_.getAppliedAging();

    // Habilita SQW a 1 Hz SIN instalar ISR en RTC_Module.
    rtc_.enableSqw1Hz();
    gps_.beginNmeaOnly();
    
    if (!pcnt_.begin(PIN_32K_RTC))
        halt("ERROR: PCNT");
    if (!pps_.begin(PIN_PPS_GPS, RISING, MIN_GAP_US,
                    CAPTURE_MAX_TICKS, INPUT))
        halt("ERROR: captura PPS");
    if (!sqw_.begin(PIN_SQW_RTC, FALLING, MIN_GAP_US,
                    CAPTURE_MAX_TICKS, INPUT_PULLUP))
        halt("ERROR: captura SQW");
    kalman_.reset();

    // Telemetria inicia Wi-Fi/TCP en su propia tarea (core 0).
    // No bloquear si el servidor esta desconectado.
    if (!telemetry_queue_.begin())
        halt("ERROR: no se pudo iniciar la cola de telemetria");
    Serial.printf("%s | aging anterior %d | aplicado %d\n",
                  device_id_, int(aging_anterior), int(aging_aplicado_));
    Serial.println("PCNT/PPS/SQW listos. Tres relojes con referencias REALES.");
}
