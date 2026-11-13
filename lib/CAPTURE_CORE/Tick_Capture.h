#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "Pcnt_Extended.h"

struct TickEdgeSample {
    //Conteo de ticks
    uint64_t ticks = 0;
    uint64_t prev = 0;
    uint32_t diff = 0;
    //Registros totales, pendientes, descartados
    uint32_t seq = 0;
    uint32_t pending = 0;
    uint32_t discarded = 0;
    //Validación
    uint32_t at_us = 0;
    uint32_t period_us = 0;
    bool valid = false;
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
    bool begin(int pin, int edge_mode, uint32_t min_gap_us = 0, uint32_t max_elapsed_ticks = 0, int pin_mode = INPUT);
    bool consume(TickEdgeSample& out);
    bool peek(TickEdgeSample& out);
    int readElapsedTicks();
    uint32_t sequence() const;
private:
    static void IRAM_ATTR router(void* arg);
    void IRAM_ATTR onPeriod();
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    TickEdgeSample last_;
    uint32_t min_gap_us_ = 0;
    uint32_t max_elapsed_ticks_= 0;
    bool started_ = false;
};
