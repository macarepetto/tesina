#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "Tick_Capture.h"

struct JsonLogRecord; // Se define en Json_Log_Builder.h.

struct PhysicalEventSample {
    uint64_t ticks = 0;    // Un unico instante para los tres Clock.
    uint32_t event_us = 0; // micros() al comienzo de la ISR.
    uint32_t number = 0;
    uint32_t discarded = 0; // Total descartados por cola llena.
};

// ISR: SOLO captura micros() + PCNT. No hace I2C, Kalman, JSON ni WiFi.
// Pulsador en GPIO13 contra GND; pull-up interno, flanco FALLING.
class PhysicalEventModule : public TickCaptureBase {
public:
    explicit PhysicalEventModule(PcntExtended& pcnt) : TickCaptureBase(pcnt) {}
    bool begin(int pin);
    bool consume(PhysicalEventSample& out);

    // Completa SOLO los campos propios de la pulsacion en un registro ya creado.
    // Se ejecuta fuera de ISR, antes de consultar el estado Kalman.
    void fillRecord(JsonLogRecord& record, const PhysicalEventSample& event) const;

    // Completa la antiguedad del ultimo PPS, si peek() devolvio una muestra.
    // Se llama despues de consultar Kalman, igual que en el main anterior.
    void fillPpsAge(JsonLogRecord& record, const PhysicalEventSample& event,
                    const TickEdgeSample& last_pps) const;
private:
    static constexpr uint32_t DEBOUNCE_US = 120000UL; // Del modulo antiguo.
    static constexpr uint8_t CAPACITY = 8;
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    PhysicalEventSample pending_[CAPACITY];
    volatile uint8_t head_ = 0;
    volatile uint8_t count_ = 0;
    volatile uint32_t total_ = 0;
    volatile uint32_t discarded_ = 0;
    volatile uint32_t last_event_us_ = 0;
    bool started_ = false;
    static void IRAM_ATTR router(void* arg);
    void IRAM_ATTR onInterrupt();
};
