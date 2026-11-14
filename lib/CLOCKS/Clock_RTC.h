#pragma once

#include "Clock.h"
#include "Tick_Capture.h"
#include "RTC_Module.h"

// Hora del calendario DS3231, interpolada desde cada flanco SQW
// usando el PCNT de 32,768 kHz.
class ClockRTC : public Clock {
public:
    ClockRTC(PcntExtended& counter, PeriodicTickCapture& sqw, RTC_Module& rtc)
        : Clock(counter), sqw_(sqw), rtc_(rtc) {}

    // Llamar frecuentemente desde loop(), NUNCA desde una ISR.
    // Es el único consumidor de sqw.consume().
    bool update();

    bool getTimeAtTicks(uint64_t ticks, int64_t& time_us) override;

    // Llamar después de escribir/ajustar el calendario RTC.
    void invalidate() { has_reference_ = false; }

private:
    PeriodicTickCapture& sqw_;
    RTC_Module& rtc_;
    bool has_reference_ = false;
    uint32_t ref_seq_ = 0;
    uint64_t ref_ticks_ = 0;
    int64_t ref_time_us_ = 0;
};
