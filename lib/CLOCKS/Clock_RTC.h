#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "Clock.h"
#include "Tick_Capture.h"
#include "RTC_Module.h"

// Port fiel de RTC_SQW_Clock_Module: la FECHA/HORA viene del DS3231
// por I2C, se ancla al SQW, y sólo la fracción proviene del PCNT de 32k.
// No usa GNSS, no entrena Kalman, no ajusta el RTC.
class ClockRTC : public Clock {
public:
    ClockRTC(PcntExtended& counter, PeriodicTickCapture& sqw, RTC_Module& rtc)
        : Clock(counter), sqw_(sqw), rtc_(rtc) {}

    bool begin(int pin_rtc_sqw);
    void invalidateOnCalendarWrite();
    bool update();
    bool getTimeAtTicks(uint64_t ticks, int64_t& time_us) override;

    // Estadísticas/detección sin cambiar el criterio del antiguo módulo.
    const char* lastState() const { return state_; }
    bool anchored() const { return anchor_valid_; }
    uint32_t lastSeq() const { return last_good_seq_; }

private:
    static constexpr uint32_t MAX_CALENDAR_READ_DELAY_US = 250000U;
    static constexpr int32_t MAX_TICK_ERROR_PER_RTC_SECOND = 10;
    static constexpr uint64_t MAX_ELAPSED_TICKS = 2ULL * TICKS_PER_SECOND;

    PeriodicTickCapture& sqw_;
    RTC_Module& rtc_;
    bool started_ = false;
    uint32_t reject_through_seq_ = 0;
    uint32_t last_good_seq_ = 0;
    uint64_t last_good_ticks_ = 0;
    int64_t last_good_epoch_s_ = 0;

    bool anchor_valid_ = false;
    uint64_t anchor_ticks_ = 0;
    int64_t anchor_epoch_s_ = 0;
    const char* state_ = "RTC_SQW_INICIO";

    static bool setDs3231Sqw1Hz();
};
