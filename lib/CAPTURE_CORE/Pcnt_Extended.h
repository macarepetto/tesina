#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "driver/pcnt.h" 

class PcntExtended {
public:
    PcntExtended() = default;
    bool begin(int pulse_gpio);
    bool started() const { return started_; }

    // Únicamente desde ISR de GPIO (PPS, SQW, pulsador).
    uint64_t IRAM_ATTR readFromISR();
    // Únicamente desde loop()/tarea normal.
    uint64_t read();

private:
    static constexpr pcnt_unit_t UNIT = PCNT_UNIT_0;
    static constexpr int16_t HIGH_LIMIT = 30000;
    static constexpr int16_t LOW_LIMIT = -30000;
    static constexpr uint32_t UNIT_MASK = (1UL << PCNT_UNIT_0);

    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    volatile uint64_t base_ = 0;
    bool started_ = false;
    pcnt_isr_handle_t intr_handle_ = nullptr;

    static void IRAM_ATTR overflowIsr(void* arg);
    bool IRAM_ATTR consumeOverflowLocked();
    uint64_t IRAM_ATTR readLocked();
};
