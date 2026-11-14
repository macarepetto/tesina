#pragma once

#include "Clock.h"
#include "Clock_RTC.h"
#include "Kalman_Filter.h"

// Hora RTC/SQW corregida por la deriva estimada por Kalman.
// NO entrena al filtro: solo consulta su estado actual.
class ClockKalman : public Clock {
public:
    ClockKalman(PcntExtended& counter, ClockRTC& rtc_clock, Kalman_Filter& kalman)
        : Clock(counter), rtc_clock_(rtc_clock), kalman_(kalman) {}

    // Llamar cuando se establezca la referencia inicial y se resetee Kalman.
    void setReference() {
        phi_start_ = kalman_.getState().phi;
        has_reference_ = true;
    }

    void invalidate() { has_reference_ = false; }

    bool getTimeAtTicks(uint64_t ticks, int64_t& time_us) override;

private:
    ClockRTC& rtc_clock_;
    Kalman_Filter& kalman_;
    bool has_reference_ = false;
    double phi_start_ = 0.0;
};
