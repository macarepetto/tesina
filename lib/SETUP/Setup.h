#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "Pcnt_Extended.h"
#include "Tick_Capture.h"
#include "GPS_Module.h"
#include "RTC_Module.h"
#include "Kalman_Filter.h"
#include "Clock_GNSS.h"
#include "Clock_RTC.h"
#include "Clock_Kalman.h"
#include "WiFi_Module.h"
#include "Telemetry_Queue_Module.h"

// Composicion de componentes de los 3 relojes y la telemetria.
// No procesa PPS, no actualiza Kalman ni construye timestamps.
class Setup {
public:
    static constexpr int PIN_32K_RTC = 18;
    static constexpr int PIN_PPS_GPS = 27;
    static constexpr int PIN_SQW_RTC = 19;
    static constexpr int PIN_GPS_RX = 16;
    static constexpr int PIN_GPS_TX = 17;
    static constexpr long GPS_BAUD = 9600;
    static constexpr int8_t AGING_VALUE = -13;
    static constexpr uint32_t CAPTURE_MAX_TICKS = 49152UL;
    static constexpr uint32_t MIN_GAP_US = 800000UL;
    static constexpr const char* WIFI_SSID = "PATAN";
    static constexpr const char* WIFI_PASS = "autoslocos";
    static constexpr const char* SERVER_IP = "192.168.0.119";
    static constexpr uint16_t SERVER_PORT = 8080;

    explicit Setup(const char* device_id);
    Setup(const Setup&) = delete;
    Setup& operator=(const Setup&) = delete;

    // Inicializa hardware y arranca la tarea de telemetria en core 0.
    void begin();

    const char* deviceId() const { return device_id_; }
    int8_t agingApplied() const { return aging_aplicado_; }

    PcntExtended& pcnt() { return pcnt_; }
    PeriodicTickCapture& pps() { return pps_; }
    PeriodicTickCapture& sqw() { return sqw_; }
    GPS_Module& gps() { return gps_; }
    RTC_Module& rtc() { return rtc_; }
    Kalman_Filter& kalman() { return kalman_; }
    ClockGNSS& clockGnss() { return clock_gnss_; }
    ClockRTC& clockRtc() { return clock_rtc_; }
    ClockKalman& clockKalman() { return clock_kalman_; }
    WiFi_Module& wifi() { return wifi_; }
    Telemetry_Queue_Module& telemetryQueue() { return telemetry_queue_; }

private:
    static void halt(const char* message);
    const char* device_id_;
    int8_t aging_aplicado_ = 0;

    // Declarar en orden de dependencias: las capturas comparten UN PCNT.
    PcntExtended pcnt_;
    PeriodicTickCapture pps_;
    PeriodicTickCapture sqw_;
    GPS_Module gps_;
    RTC_Module rtc_;
    Kalman_Filter kalman_;
    ClockGNSS clock_gnss_;
    ClockRTC clock_rtc_;
    ClockKalman clock_kalman_;
    WiFi_Module wifi_;
    Telemetry_Queue_Module telemetry_queue_;
};