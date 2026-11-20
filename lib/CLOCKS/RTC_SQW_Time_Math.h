#ifndef RTC_SQW_TIME_MATH_H
#define RTC_SQW_TIME_MATH_H

#include <stdint.h>

// Conversion entera sin punto flotante; redondeo al microsegundo mas cercano.
// No confundir el formato de 1 us con la resolucion fisica de 30.52 us.
namespace RTC_SQW_Time_Math {

static constexpr uint32_t TICKS_PER_SECOND = 32768U;
static constexpr uint32_t MAX_ELAPSED_TICKS = 2U * TICKS_PER_SECOND;

inline uint64_t ticksToMicroseconds(uint32_t ticks) {
    return ((uint64_t)ticks * 1000000ULL + TICKS_PER_SECOND / 2U) /
           TICKS_PER_SECOND;
}

inline bool convert(uint32_t start_ticks, uint32_t event_ticks,
                    int64_t rtc_epoch_s, int64_t& result_epoch_us,
                    uint32_t& elapsed_ticks) {
    elapsed_ticks = event_ticks - start_ticks;  // modulo uint32_t
    if (elapsed_ticks > MAX_ELAPSED_TICKS) return false;
    if (rtc_epoch_s <= 0) return false;
    result_epoch_us = rtc_epoch_s * 1000000LL +
                      (int64_t)ticksToMicroseconds(elapsed_ticks);
    return true;
}

}  // namespace RTC_SQW_Time_Math

#endif
