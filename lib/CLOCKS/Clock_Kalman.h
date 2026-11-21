#pragma once
#include "Clock.h"
#include "Clock_RTC.h"
#include "Kalman_Filter.h"

// No ejecuta predict() ni updatePhi(). Solo aplica la corrección
// acumulada de phi AL RELOJ REAL RTC/SQW.
class ClockKalman : public Clock {
public:
    ClockKalman(PcntExtended& counter, ClockRTC& rtc, Kalman_Filter& kalman)
        : Clock(counter), rtc_(rtc), kalman_(kalman) {}

    void setReference() {
        phi_start_ = kalman_.getState().phi;
        has_reference_ = true;
        has_event_state_ = false;
    }

    void captureStateAtTicks(uint64_t ticks) {
        event_ticks_ = ticks;
        event_phi_ = kalman_.getState().phi;
        has_event_state_ = true;
    }

    void invalidate() {
        has_reference_ = false;
        has_event_state_ = false;
    }

    bool getTimeAtTicks(uint64_t ticks, int64_t& time_us) override;

private:
    ClockRTC& rtc_;
    Kalman_Filter& kalman_;
    bool has_reference_ = false;
    double phi_start_ = 0.0;
    bool has_event_state_ = false;
    uint64_t event_ticks_ = 0;
    double event_phi_ = 0.0;
};
