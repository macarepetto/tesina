#include "Tick_Capture.h"

bool PeriodicTickCapture::begin(int pin, int edge_mode, uint32_t min_gap_us, int pin_mode) {
    if (started_) return true;
    if (!counter_.started()) return false;
    min_gap_us_ = min_gap_us;
    pinMode(pin, pin_mode);
    attachInterruptArg((uint8_t)pin, &PeriodicTickCapture::router, this, edge_mode);
    started_ = true;
    return true;
}

void IRAM_ATTR PeriodicTickCapture::router(void* arg) {
    static_cast<PeriodicTickCapture*>(arg)->onEdge();
}

void IRAM_ATTR PeriodicTickCapture::onEdge() {
    const uint32_t now_us = micros();
    // No sostener el lock del capturador mientras tomamos el lock del PCNT.
    // Sólo esta ISR modifica las muestras de esta instancia.
    if (last_.seq && uint32_t(now_us - last_.at_us) < min_gap_us_) return;
    const uint64_t ticks = ticksAtInterrupt();
    portENTER_CRITICAL_ISR(&mux_);
    if (last_.pending) ++last_.discarded;
    last_.period_us = last_.seq ? uint32_t(now_us - last_.at_us) : 0;
    last_.at_us = now_us;
    last_.ticks = ticks;
    ++last_.seq;
    ++last_.pending;
    portEXIT_CRITICAL_ISR(&mux_);
}

bool PeriodicTickCapture::consume(TickEdgeSample& out) {
    portENTER_CRITICAL(&mux_);
    const bool available = last_.pending != 0;
    if (available) {
        out = last_;
        last_.pending = 0;
    }
    portEXIT_CRITICAL(&mux_);
    return available;
}

bool PeriodicTickCapture::snapshot(TickEdgeSample& out) const {
    auto* self = const_cast<PeriodicTickCapture*>(this);
    portENTER_CRITICAL(&self->mux_);
    out = self->last_;
    portEXIT_CRITICAL(&self->mux_);
    return out.seq != 0;
}

bool IRAM_ATTR PeriodicTickCapture::snapshotFromISR(TickEdgeSample& out) const {
    auto* self = const_cast<PeriodicTickCapture*>(this);
    portENTER_CRITICAL_ISR(&self->mux_);
    out = self->last_;
    portEXIT_CRITICAL_ISR(&self->mux_);
    return out.seq != 0;
}

uint32_t PeriodicTickCapture::sequence() const {
    TickEdgeSample s;
    return snapshot(s) ? s.seq : 0;
}

bool ButtonTickCapture::begin(int pin, uint32_t debounce_us) {
    if (started_) return true;
    if (!counter_.started()) return false;
    debounce_us_ = debounce_us;
    pinMode(pin, INPUT_PULLUP);
    attachInterruptArg((uint8_t)pin, &ButtonTickCapture::router, this, FALLING);
    started_ = true;
    return true;
}

void IRAM_ATTR ButtonTickCapture::router(void* arg) {
    static_cast<ButtonTickCapture*>(arg)->onButton();
}

void IRAM_ATTR ButtonTickCapture::onButton() {
    const uint32_t now_us = micros();
    if (last_accepted_us_ && uint32_t(now_us - last_accepted_us_) < debounce_us_) return;
    last_accepted_us_ = now_us;

    ButtonTickSample event;
    event.event_us = now_us;
    event.event_ticks = ticksAtInterrupt();
    event.captured = true;
    event.has_pps = pps_.snapshotFromISR(event.last_pps);
    event.has_sqw = sqw_.snapshotFromISR(event.last_sqw);
    // El delta es SIEMPRE desde la última referencia, NO desde el último botón.
    if (event.has_pps && event.event_ticks >= event.last_pps.ticks) {
        event.ticks_since_pps = event.event_ticks - event.last_pps.ticks;
    } else {
        event.has_pps = false;
    }
    if (event.has_sqw && event.event_ticks >= event.last_sqw.ticks) {
        event.ticks_since_sqw = event.event_ticks - event.last_sqw.ticks;
    } else {
        event.has_sqw = false;
    }

    portENTER_CRITICAL_ISR(&mux_);
    event.number = ++event_count_;
    const uint8_t next = (head_ + 1) % QUEUE_SIZE;
    if (next != tail_) {
        queue_[head_] = event;
        head_ = next;
    } else {
        ++dropped_;
    }
    portEXIT_CRITICAL_ISR(&mux_);
}

bool ButtonTickCapture::consume(ButtonTickSample& out, uint32_t& dropped) {
    portENTER_CRITICAL(&mux_);
    const bool available = tail_ != head_;
    if (available) {
        out = queue_[tail_];
        tail_ = (tail_ + 1) % QUEUE_SIZE;
    }
    dropped = dropped_;
    portEXIT_CRITICAL(&mux_);
    return available;
}
