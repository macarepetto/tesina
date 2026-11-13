#include "Tick_Capture.h"

bool PeriodicTickCapture::begin(int pin, int edge_mode, uint32_t min_gap_us, uint32_t max_elapsed_ticks, int pin_mode) {
    if (started_) return true;
    if (!counter_.started()) return false;
    min_gap_us_ = min_gap_us;
    pinMode(pin, pin_mode);
    max_elapsed_ticks_ = max_elapsed_ticks;
    attachInterruptArg((uint8_t)pin, &PeriodicTickCapture::router, this, edge_mode);
    started_ = true;
    last_.valid = false;
    return true;
}

void IRAM_ATTR PeriodicTickCapture::router(void* arg) {
    static_cast<PeriodicTickCapture*>(arg)->onPeriod();
}

/// AGREGAR VALIDACION DE HOLDOVER
/// 
void IRAM_ATTR PeriodicTickCapture::onPeriod() {
    const uint32_t now_us = micros();
    // No sostener el lock del capturador mientras tomamos el lock del PCNT.
    // Sólo esta ISR modifica las muestras de esta instancia.
    if (last_.seq && uint32_t(now_us - last_.at_us) < min_gap_us_) return;
    const uint64_t ticks = ticksAtInterrupt();

    portENTER_CRITICAL_ISR(&mux_);
    //Validación
    last_.period_us = last_.seq ? uint32_t(now_us - last_.at_us) : 0;
    last_.at_us = now_us;
    //Ticks
    last_.ticks = ticks;
    //solo actualizar el diff cuando no es el 1ro
    last_.diff = last_.ticks - last_.prev;
    last_.prev = last_.ticks;

    //Registros totales, pendientes y descartados
    if (last_.pending) ++last_.discarded;
    
    if (last_.diff>max_elapsed_ticks_){
        last_.seq = 0;
        last_.pending = 0;
    } else {
        ++last_.seq;
        ++last_.pending;
    }
    last_.valid = true;
    portEXIT_CRITICAL_ISR(&mux_);
}

int PeriodicTickCapture::readElapsedTicks() {
    int diff = -1;
    TickEdgeSample last;
    // No sostener el lock del capturador mientras tomamos el lock del PCNT.
    // Sólo esta ISR modifica las muestras de esta instancia.
    const uint64_t ticks = ticksAtInterrupt();
    if (peek(last)){
        uint32_t aux = ticks - last.ticks;
        //Verifica que no sea un registro viejo.
        if (aux<max_elapsed_ticks_){
            diff = aux;
        }
    };
    return diff;
}

bool PeriodicTickCapture::consume(TickEdgeSample& out) {

    portENTER_CRITICAL(&mux_);
    //Hay datos disponibles
    const bool available = last_.pending != 0;
    if (available) {
        //Devuelve una copia de last_ y resetea el pending
        out = last_;
        last_.pending = 0;
    }
    portEXIT_CRITICAL(&mux_);
    //Devuelve true si había datos disponibles
    return available;
}

bool PeriodicTickCapture::peek(TickEdgeSample& out) {

    portENTER_CRITICAL(&mux_);
    //Hay datos disponibles
    const bool available = last_.valid;
    if (available) {
        //Devuelve una copia de last_ y no consume
        out = last_;
    }
    portEXIT_CRITICAL(&mux_);
    //Devuelve true si había datos disponibles
    return available;
}

