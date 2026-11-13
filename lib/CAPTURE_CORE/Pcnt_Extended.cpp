#include "Pcnt_Extended.h"
#include "esp_intr_alloc.h"
#include "soc/pcnt_struct.h"

// El hardware reinicia el contador al alcanzar +30000.
// Se usa una sola ISR del periférico, protegida con el mismo mux que las lecturas.
bool IRAM_ATTR PcntExtended::consumeOverflowLocked() {
    if ((PCNT.int_st.val & UNIT_MASK) == 0) return false;
    PCNT.int_clr.val = UNIT_MASK;
    base_ += HIGH_LIMIT;
    return true;
}

uint64_t IRAM_ATTR PcntExtended::readLocked() {
    // Un desborde puede estar pendiente aunque todavía no se haya atendido su ISR.
    consumeOverflowLocked();
    uint16_t raw = static_cast<uint16_t>(PCNT.cnt_unit[PCNT_UNIT_0].cnt_val);
    // Si ocurrió durante la lectura, incorporar ese desborde y volver a leer.
    if (consumeOverflowLocked()) {
        raw = static_cast<uint16_t>(PCNT.cnt_unit[PCNT_UNIT_0].cnt_val);
    }
    return base_ + raw;
}

void IRAM_ATTR PcntExtended::overflowIsr(void* arg) {
    auto* self = static_cast<PcntExtended*>(arg);
    portENTER_CRITICAL_ISR(&self->mux_);
    self->consumeOverflowLocked();
    portEXIT_CRITICAL_ISR(&self->mux_);
}

/// 
/// VER DE CAMBIAR LA FORMA DE PASAR EL CANAL (NO HARDCODEAR)
bool PcntExtended::begin(int pulse_gpio) {
    if (started_) return true;
    pinMode(pulse_gpio, INPUT_PULLUP);
    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = pulse_gpio;
    cfg.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    cfg.pos_mode = PCNT_COUNT_INC;
    cfg.neg_mode = PCNT_COUNT_DIS;
    cfg.lctrl_mode = PCNT_MODE_KEEP;
    cfg.hctrl_mode = PCNT_MODE_KEEP;
    cfg.counter_h_lim = HIGH_LIMIT;
    cfg.counter_l_lim = LOW_LIMIT;
    cfg.unit = UNIT;
    cfg.channel = PCNT_CHANNEL_0;
    if (pcnt_unit_config(&cfg) != ESP_OK) return false;
    if (pcnt_filter_disable(UNIT) != ESP_OK) return false;
    if (pcnt_counter_pause(UNIT) != ESP_OK) return false;
    if (pcnt_counter_clear(UNIT) != ESP_OK) return false;

    portENTER_CRITICAL(&mux_);
    base_ = 0;
    portEXIT_CRITICAL(&mux_);

    if (pcnt_event_enable(UNIT, PCNT_EVT_H_LIM) != ESP_OK) return false;
    // API directa, en lugar del servicio PCNT utilizado en la versión anterior.
    // No deben convivir ambas instalaciones sobre la misma unidad.
    if (pcnt_isr_register(&PcntExtended::overflowIsr, this,
                          ESP_INTR_FLAG_IRAM, &intr_handle_) != ESP_OK) return false;
    if (pcnt_intr_enable(UNIT) != ESP_OK) return false;
    if (pcnt_counter_resume(UNIT) != ESP_OK) return false;
    started_ = true;
    return true;
}

uint64_t IRAM_ATTR PcntExtended::readFromISR() {
    portENTER_CRITICAL_ISR(&mux_);
    const uint64_t ticks = readLocked();
    portEXIT_CRITICAL_ISR(&mux_);
    return ticks;
}

uint64_t PcntExtended::read() {
    portENTER_CRITICAL(&mux_);
    const uint64_t ticks = readLocked();
    portEXIT_CRITICAL(&mux_);
    return ticks;
}
