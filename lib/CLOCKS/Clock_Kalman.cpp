#include "Clock_Kalman.h"
#include <math.h>

bool ClockKalman::getTimeAtTicks(uint64_t ticks, int64_t& time_us) {
    if (!has_reference_) return false;

    int64_t rtc_time_us = 0;
    if (!rtc_clock_.getTimeAtTicks(ticks, rtc_time_us)) return false;

    // phi es un OFFSET ACUMULADO EN TICKS, tal como lo calcula el main.
    // Se descuenta el cambio desde el origen; no se cambia el calendario RTC.
    const double drift_ticks = kalman_.getState().phi - phi_start_;
    if (!isfinite(drift_ticks)) return false;
    const int64_t correction_us = static_cast<int64_t>(
        llround(drift_ticks * (1000000.0 / 32768.0))
    );

    time_us = rtc_time_us - correction_us;
    return true;
}
