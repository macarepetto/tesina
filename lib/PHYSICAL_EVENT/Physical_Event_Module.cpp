#include "Physical_Event_Module.h"
#include "Json_Log_Builder.h"

bool PhysicalEventModule::begin(int pin) {
    if (started_) return true;
    if (!counter_.started()) return false;
    pinMode(pin, INPUT_PULLUP);
    attachInterruptArg(uint8_t(pin), &PhysicalEventModule::router,
                       this, FALLING);
    started_ = true;
    return true;
}

void IRAM_ATTR PhysicalEventModule::router(void* arg) {
    static_cast<PhysicalEventModule*>(arg)->onInterrupt();
}

void IRAM_ATTR PhysicalEventModule::onInterrupt() {
    const uint32_t at_us = micros();
    // Solo se acepta una pulsacion por cada 120 ms, igual que antes.
    if (total_ && uint32_t(at_us - last_event_us_) < DEBOUNCE_US) return;
    const uint64_t ticks = ticksAtInterrupt();

    portENTER_CRITICAL_ISR(&mux_);
    // Solo esta ISR modifica total_ y last_event_us_.
    ++total_;
    last_event_us_ = at_us;
    if (count_ < CAPACITY) {
        const uint8_t slot = uint8_t((head_ + count_) % CAPACITY);
        pending_[slot].ticks = ticks;
        pending_[slot].event_us = at_us;
        pending_[slot].number = total_;
        pending_[slot].discarded = discarded_;
        ++count_;
    } else {
        ++discarded_;
    }
    portEXIT_CRITICAL_ISR(&mux_);
}

bool PhysicalEventModule::consume(PhysicalEventSample& out) {
    portENTER_CRITICAL(&mux_);
    const bool available = count_ != 0;
    if (available) {
        out = pending_[head_];
        out.discarded = discarded_; // Contabiliza tambien descartes posteriores.
        head_ = uint8_t((head_ + 1) % CAPACITY);
        --count_;
    }
    portEXIT_CRITICAL(&mux_);
    return available;
}

// MISMO armado que estaba en publicarEventosFisicos() del main.
// No se consulta RTC, GNSS, SQW, Kalman, ni se envia telemetria.
void PhysicalEventModule::fillRecord(JsonLogRecord& record,
                                     const PhysicalEventSample& event) const {
    record.detalle = "PULSADOR";
    record.event_captured = true;
    record.event_num = event.number;
    record.event_us = event.event_us;
    record.event_delay_to_log_us = uint32_t(micros() - event.event_us);
    record.event_discarded = event.discarded;
}

void PhysicalEventModule::fillPpsAge(JsonLogRecord& record,
                                     const PhysicalEventSample& event,
                                     const TickEdgeSample& last_pps) const {
    if (event.ticks >= last_pps.ticks &&
        event.ticks - last_pps.ticks <= 49152ULL) {
        const uint32_t age_us = uint32_t(event.event_us - last_pps.at_us);
        if (age_us <= 1500000UL) {
            record.event_pps_age_valid = true;
            record.ms_since_last_pps = age_us / 1000UL;
        }
    }
}
