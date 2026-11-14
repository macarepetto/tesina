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

// Composicion de componentes para la prueba de los 3 relojes.
// No procesa PPS, no actualiza Kalman y no construye timestamps en el setup.
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

    explicit Setup(const char* device_id);
    Setup(const Setup&) = delete;
    Setup& operator=(const Setup&) = delete;

    // Inicializa solamente el hardware requerido en el test.
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
};