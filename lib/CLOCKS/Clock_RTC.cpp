#include "Clock_RTC.h"
#include <Wire.h>

namespace {
constexpr uint8_t DS3231_I2C_ADDRESS = 0x68;
constexpr uint8_t DS3231_CONTROL_REGISTER = 0x0E;
constexpr uint8_t SQW_MODE_MASK = 0x1C;
}

// Mismo código de configuración de RTC_SQW_Clock_Module.
bool ClockRTC::setDs3231Sqw1Hz() {
    Wire.beginTransmission(DS3231_I2C_ADDRESS);
    Wire.write(DS3231_CONTROL_REGISTER);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(DS3231_I2C_ADDRESS, (uint8_t)1) != 1) return false;
    const uint8_t control = uint8_t(Wire.read());
    const uint8_t new_control = uint8_t(control & ~SQW_MODE_MASK);
    Wire.beginTransmission(DS3231_I2C_ADDRESS);
    Wire.write(DS3231_CONTROL_REGISTER);
    Wire.write(new_control);
    return Wire.endTransmission() == 0;
}

bool ClockRTC::begin(int pin_rtc_sqw) {
    if (started_) return true;
    if (pin_rtc_sqw < 0) return false;
    if (!setDs3231Sqw1Hz()) return false;

    anchor_valid_ = false;
    last_good_seq_ = 0;
    // Se conserva FALLING, min_gap 0, INPUT_PULLUP del módulo anterior.
    if (!sqw_.begin(pin_rtc_sqw, FALLING, 0, 0, INPUT_PULLUP)) return false;
    started_ = true;
    return true;
}

void ClockRTC::invalidateOnCalendarWrite() {
    anchor_valid_ = false;
    last_good_seq_ = 0;
    TickEdgeSample latest;
    reject_through_seq_ = sqw_.peek(latest) ? latest.seq : 0;
}

bool ClockRTC::update() {
    if (!started_) return false;
    TickEdgeSample sample;
    if (!sqw_.consume(sample)) return false;
    if (sample.seq <= reject_through_seq_) return false;

    const uint32_t capture_age_us = uint32_t(micros() - sample.at_us);
    const char* state = "RTC_SQW_INICIO";
    int64_t epoch_s = 0;
    bool valid = sample.valid;

    if (!sample.valid) {
        state = "RTC_SQW_PCNT_FALLO";
    } else if (capture_age_us > MAX_CALENDAR_READ_DELAY_US) {
        valid = false;
        state = "RTC_SQW_LECTURA_TARDIA";
    } else if (!rtc_.readEpoch(epoch_s)) {
        valid = false;
        state = "RTC_SQW_CALENDARIO_INVALIDO";
    } else {
        TickEdgeSample latest;
        if (!sqw_.peek(latest) || sample.seq != latest.seq) {
            valid = false;
            state = "RTC_SQW_CAMBIO_DURANTE_LECTURA";
        }
    }

    if (valid && last_good_seq_ > 0 && sample.seq == last_good_seq_ + 1) {
        const uint64_t window_ticks = sample.ticks - last_good_ticks_;
        const int64_t tick_error =
            int64_t(window_ticks) - int64_t(TICKS_PER_SECOND);
        if (epoch_s != last_good_epoch_s_ + 1LL) {
            valid = false;
            state = "RTC_SQW_SEGUNDOS_NO_CONSECUTIVOS";
        } else if (tick_error > MAX_TICK_ERROR_PER_RTC_SECOND ||
                   tick_error < -MAX_TICK_ERROR_PER_RTC_SECOND) {
            valid = false;
            state = "RTC_SQW_TICKS_INCONSISTENTES";
        } else {
            state = "RTC_SQW_OK";
        }
    } else if (valid && last_good_seq_ > 0) {
        state = "RTC_SQW_REANCLA_TRAS_SALTO";
    }

    if (valid) {
        // Hora de CALENDARIO leída por I2C. Nunca origen GNSS.
        anchor_ticks_ = sample.ticks;
        anchor_epoch_s_ = epoch_s;
        anchor_valid_ = true;
        last_good_seq_ = sample.seq;
        last_good_ticks_ = sample.ticks;
        last_good_epoch_s_ = epoch_s;
    } else {
        anchor_valid_ = false;
        last_good_seq_ = 0;
    }
    state_ = state;
    return valid;
}

bool ClockRTC::getTimeAtTicks(uint64_t ticks, int64_t& time_us) {
    if (!anchor_valid_ || ticks < anchor_ticks_ ||
        ticks - anchor_ticks_ > MAX_ELAPSED_TICKS ||
        anchor_epoch_s_ <= 0) return false;
    // Igual a RTC_SQW_Time_Math::convert: redondeo entero al us.
    time_us = anchor_epoch_s_ * 1000000LL +
              ticksToUs(ticks - anchor_ticks_);
    return true;
}
