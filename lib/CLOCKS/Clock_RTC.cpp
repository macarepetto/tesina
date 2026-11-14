#include "Clock_RTC.h"

bool ClockRTC::update() {
    TickEdgeSample sample;
    if (!sqw_.consume(sample)) return false;

    // Un nuevo flanco invalida el anclaje anterior hasta que se lea
    // correctamente el calendario. No usar una muestra atrasada.
    has_reference_ = false;
    if (!sample.valid || sample.seq == 0 || sample.pending != 1 ||
        uint32_t(micros() - sample.at_us) > 250000UL) {
        return false;
    }

    int64_t epoch_seconds = 0;
    if (!rtc_.readEpoch(epoch_seconds) || epoch_seconds <= 0) {
        return false;
    }

    // Comprobar que no llegó otro SQW durante la operación I2C.
    TickEdgeSample latest;
    if (!sqw_.peek(latest) || latest.seq != sample.seq ||
        latest.ticks != sample.ticks ||
        uint32_t(micros() - sample.at_us) > 250000UL) {
        return false;
    }

    ref_seq_ = sample.seq;
    ref_ticks_ = sample.ticks;
    ref_time_us_ = epoch_seconds * 1000000LL;
    has_reference_ = true;
    return true;
}

bool ClockRTC::getTimeAtTicks(uint64_t ticks, int64_t& time_us) {
    if (!has_reference_) return false;

    TickEdgeSample latest;
    if (!sqw_.peek(latest) || latest.seq != ref_seq_ ||
        latest.ticks != ref_ticks_) {
        return false;
    }

    if (ticks < ref_ticks_ || ticks - ref_ticks_ > MAX_EDGE_AGE_TICKS) {
        return false;
    }

    time_us = ref_time_us_ + ticksToUs(ticks - ref_ticks_);
    return true;
}
