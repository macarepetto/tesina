#include "Setup.h"

#include "PPS_Capture.h"
#include "RTC_SQW_Clock_Module.h"
#include "Physical_Event_Module.h"

namespace {
constexpr char WIFI_SSID[] = "PATAN";
constexpr char WIFI_PASS[] = "autoslocos";
constexpr char SERVER_IP[] = "192.168.0.119";
constexpr uint16_t SERVER_PORT = 8080;
}

Setup::Setup(const char* device_id)
    : device_id_(device_id),
      gps_(Serial2, PIN_GPS_RX, PIN_GPS_TX, PIN_PPS_GPS, GPS_BAUD),
      rtc_(),
      wifi_(WIFI_SSID, WIFI_PASS, SERVER_IP, SERVER_PORT, device_id),
      telemetry_(wifi_),
      kalman_(),
      clocks_(),
      diagnostics_(gps_, rtc_, clocks_) {
}

void Setup::halt(const char* message) {
    Serial.println(message);
    while (true) {
        delay(1000);
    }
}

void Setup::begin() {
    Serial.begin(115200);
    delay(300);

    if (!rtc_.begin()) {
        halt("ERROR: no se detecto el DS3231 por I2C");
    }

    gps_.beginNmeaOnly();
    Serial.println("GNSS NMEA iniciado por GPS_Module sobre Serial2");
    Serial.print("GPS RX GPIO: ");
    Serial.println(PIN_GPS_RX);

    int8_t aging_anterior = 0;
    if (!rtc_.readAging(aging_anterior)) {
        halt("ERROR: no se pudo leer el aging del DS3231");
    }

    Serial.printf("Aging anterior: %d | Aging solicitado: %d\n",
                  (int)aging_anterior, (int)AGING_VALUE);

    if (!rtc_.applyAgingValue(AGING_VALUE)) {
        halt("ERROR: no se pudo aplicar/verificar el aging");
    }
    aging_aplicado_ = rtc_.getAppliedAging();
    Serial.printf("Aging aplicado y verificado: %d\n", (int)aging_aplicado_);

    PPS_Capture::begin(PIN_32K_RTC, PIN_PPS_GPS);
    if (!RTC_SQW_Clock::begin(PIN_SQW_RTC)) {
        Serial.println("ADVERTENCIA: RTC/SQW no disponible; reloj_rtc_calendario_sqw sera null");
    }

    if (!telemetry_.begin()) {
        halt("ERROR: no se pudo crear la cola/tarea de telemetria");
    }
    Serial.println("PCNT iniciado. Telemetria en CORE 0. Esperando PPS...");

    Physical_Event::begin(PIN_PULSADOR);
    Serial.println("Evento fisico iniciado en GPIO13");
}
