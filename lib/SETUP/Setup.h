#ifndef TESINA_SETUP_H
#define TESINA_SETUP_H

#include <Arduino.h>
#include <stdint.h>

#include "GPS_Module.h"
#include "RTC_Module.h"
#include "WiFi_Module.h"
#include "Telemetry_Queue_Module.h"
#include "Kalman_Filter.h"
#include "Three_Clocks_Module.h"
#include "RTC_GNSS_Diagnostics_Module.h"

// Configuracion y composicion de los componentes del prototipo.
class Setup {
public:
    // Configuracion comun a ambos prototipos.
    static constexpr int PIN_32K_RTC = 18;
    static constexpr int PIN_PPS_GPS = 27;
    static constexpr int PIN_SQW_RTC = 19;
    static constexpr int PIN_GPS_RX = 16;
    static constexpr int PIN_GPS_TX = 17;
    static constexpr int PIN_PULSADOR = 13;
    static constexpr long GPS_BAUD = 9600;

    static constexpr int8_t AGING_VALUE = -13;
    static constexpr uint32_t PPS_TIME_TOLERANCE_US = 200000UL;
    static constexpr int64_t MAX_DIFF_TICKS = 10;
    static constexpr double TICK_US = 1000000.0 / 32768.0;
    static constexpr uint32_t HOLDOVER_TIMEOUT_MS = 1500;
    static constexpr uint32_t HOLDOVER_STEP_MS = 1000;
    static constexpr uint32_t BACKGROUND_TASKS_PERIOD_MS = 20;
    static constexpr uint32_t UTC_UBX_MAX_AGE_MS = 3000;
    static constexpr uint32_t RTC_SYNC_MAX_POST_PPS_US = 50000;

    // La identidad cambia entre los dos main; el resto de la configuracion es comun.
    explicit Setup(const char* device_id);
    Setup(const Setup&) = delete;
    Setup& operator=(const Setup&) = delete;

    // Hace exactamente la secuencia de inicializacion del setup() anterior.
    void begin();

    const char* deviceId() const { return device_id_; }
    int8_t agingApplied() const { return aging_aplicado_; }

    GPS_Module& gps() { return gps_; }
    RTC_Module& rtc() { return rtc_; }
    WiFi_Module& wifi() { return wifi_; }
    Telemetry_Queue_Module& telemetry() { return telemetry_; }
    Kalman_Filter& kalman() { return kalman_; }
    Three_Clocks_Module& clocks() { return clocks_; }
    RTC_GNSS_Diagnostics_Module& diagnostics() { return diagnostics_; }

private:
    static void halt(const char* message);

    const char* device_id_;
    int8_t aging_aplicado_ = 0;

    GPS_Module gps_;
    RTC_Module rtc_;
    WiFi_Module wifi_;
    Telemetry_Queue_Module telemetry_;
    Kalman_Filter kalman_;
    Three_Clocks_Module clocks_;
    RTC_GNSS_Diagnostics_Module diagnostics_;
};

#endif
