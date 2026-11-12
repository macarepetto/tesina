#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "Pcnt_Extended.h"

struct TickEdgeSample {
    uint64_t ticks = 0;
    uint32_t seq = 0;
    uint32_t pending = 0;
    uint32_t at_us = 0;
    uint32_t period_us = 0;
    uint32_t discarded = 0; // flancos sobrescritos antes de consumirlos
};

// Clase base: dos capturadores y el pulsador consultan el MISMO contador.
class TickCaptureBase {
public:
    explicit TickCaptureBase(PcntExtended& counter) : counter_(counter) {}
protected:
    uint64_t IRAM_ATTR ticksAtInterrupt() { return counter_.readFromISR(); }
    PcntExtended& counter_;
};

// Una instancia PPS (RISING) y otra SQW (FALLING).
// No hace cálculos de GNSS, I2C, horas, offset o Kalman.
class PeriodicTickCapture : public TickCaptureBase {
public:
    explicit PeriodicTickCapture(PcntExtended& counter) : TickCaptureBase(counter) {}
    bool begin(int pin, int edge_mode, uint32_t min_gap_us = 0, int pin_mode = INPUT);
    bool consume(TickEdgeSample& out);
    bool snapshot(TickEdgeSample& out) const;
    bool IRAM_ATTR snapshotFromISR(TickEdgeSample& out) const;
    uint32_t sequence() const;
private:
    static void IRAM_ATTR router(void* arg);
    void IRAM_ATTR onEdge();
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    TickEdgeSample last_;
    uint32_t min_gap_us_ = 0;
    bool started_ = false;
};

struct ButtonTickSample {
    uint32_t number = 0;
    uint32_t event_us = 0;
    uint64_t event_ticks = 0;
    bool captured = false;
    // Referencias tomadas EN el evento, no al publicarse su JSON.
    TickEdgeSample last_pps;
    TickEdgeSample last_sqw;
    bool has_pps = false;
    bool has_sqw = false;
    uint64_t ticks_since_pps = 0;
    uint64_t ticks_since_sqw = 0;
};

// La clase hermana de PeriodicTickCapture usa la misma base de adquisición.
// A diferencia de PPS/SQW, el botón ocurre en cualquier momento y
// NO reemplaza la referencia anterior: se calcula contra último PPS/SQW.
class ButtonTickCapture : public TickCaptureBase {
public:
    ButtonTickCapture(PcntExtended& counter,
                      PeriodicTickCapture& pps, PeriodicTickCapture& sqw)
        : TickCaptureBase(counter), pps_(pps), sqw_(sqw) {}
    bool begin(int pin, uint32_t debounce_us = 120000);
    bool consume(ButtonTickSample& out, uint32_t& dropped);
private:
    static constexpr uint8_t QUEUE_SIZE = 16;
    static void IRAM_ATTR router(void* arg);
    void IRAM_ATTR onButton();
    PeriodicTickCapture& pps_;
    PeriodicTickCapture& sqw_;
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    ButtonTickSample queue_[QUEUE_SIZE] = {};
    volatile uint8_t head_ = 0;
    volatile uint8_t tail_ = 0;
    volatile uint32_t event_count_ = 0;
    volatile uint32_t dropped_ = 0;
    volatile uint32_t last_accepted_us_ = 0;
    uint32_t debounce_us_ = 120000;
    bool started_ = false;
};
