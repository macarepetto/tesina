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

class Setup {
public:
    static constexpr int PIN_32K_RTC = 18;
    static constexpr int PIN_PPS_GPS = 27;
    static constexpr int PIN_SQW_RTC = 19;
    static constexpr int PIN_GPS_RX = 16;
    static constexpr int PIN_GPS_TX = 17;
    static constexpr int PIN_PULSADOR = 13;
    static constexpr long GPS_BAUD = 9600;
    static constexpr int8_t AGING_VALUE = -13;
    static constexpr const char* WIFI_SSID = "PATAN";
    static constexpr const char* WIFI_PASS = "autoslocos";
    static constexpr const char* SERVER_IP = "192.168.0.119";
    static constexpr uint16_t SERVER_PORT = 8080;

    static constexpr uint32_t PPS_TIME_TOLERANCE_US = 200000UL;
    static constexpr int64_t MAX_DIFF_TICKS = 10;
    static constexpr uint32_t HOLDOVER_TIMEOUT_MS = 1500;
    static constexpr uint32_t HOLDOVER_STEP_MS = 1000;
    static constexpr uint32_t BACKGROUND_TASKS_PERIOD_MS = 20;
    static constexpr uint32_t RTC_SYNC_MAX_POST_PPS_US = 50000;

    explicit Setup(const char* device_id);
    void begin();

    const char* deviceId() const { return device_id_; }
    int8_t agingApplied() const { return aging_aplicado_; }
    PcntExtended& pcnt() { return pcnt_; }
    PeriodicTickCapture& pps() { return pps_; }
    GPS_Module& gps() { return gps_; }
    RTC_Module& rtc() { return rtc_; }
    Kalman_Filter& kalman() { return kalman_; }
    ClockGNSS& clockGnss() { return clock_gnss_; }
    ClockRTC& clockRtc() { return clock_rtc_; }
    ClockKalman& clockKalman() { return clock_kalman_; }
    Telemetry_Queue_Module& telemetryQueue() { return telemetry_queue_; }

private:
    static void halt(const char* message);
    const char* device_id_;
    int8_t aging_aplicado_ = 0;
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
