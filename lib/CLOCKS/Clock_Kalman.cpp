#include "Clock_Kalman.h"
#include <math.h>

bool ClockKalman::getTimeAtTicks(uint64_t ticks, int64_t& time_us) {
    if (!has_reference_) return false;
    int64_t rtc_us = 0;
    if (!rtc_.getTimeAtTicks(ticks, rtc_us)) return false;

    // Conserva la convención del original: phi es offset acumulado en ticks.
    const double phi = (has_event_state_ && ticks == event_ticks_)
        ? event_phi_ : kalman_.getState().phi;
    const double drift_ticks = phi - phi_start_;
    if (!isfinite(drift_ticks)) return false;
    const int64_t correction_us =
        static_cast<int64_t>(llround(drift_ticks * (1000000.0 / 32768.0)));
    time_us = rtc_us - correction_us;
    return true;
}
