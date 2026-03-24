#include "RTC32K_Pcnt_Module.h"

volatile uint32_t RTC32K_Pcnt_Module::_ovf[PCNT_UNIT_MAX] = {0};
bool RTC32K_Pcnt_Module::_isrServiceInstalled = false;

RTC32K_Pcnt_Module::RTC32K_Pcnt_Module(int pin32k, pcnt_unit_t unit, pcnt_channel_t ch)
  : _pin(pin32k), _unit(unit), _ch(ch) {}

bool RTC32K_Pcnt_Module::begin(uint16_t filterTicks) {
  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = _pin;
  cfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
  cfg.unit           = _unit;
  cfg.channel        = _ch;

  // contar solo flanco positivo
  cfg.pos_mode = PCNT_COUNT_INC;
  cfg.neg_mode = PCNT_COUNT_DIS;

  cfg.lctrl_mode = PCNT_MODE_KEEP;
  cfg.hctrl_mode = PCNT_MODE_KEEP;

  // 16-bit signed: máximo 32767. Con 32kHz en 1s siempre llegamos al límite.
  cfg.counter_h_lim = 32767;
  cfg.counter_l_lim = 0;

  if (pcnt_unit_config(&cfg) != ESP_OK) return false;

  // ========= FILTRO: solo si > 0 =========
  if (filterTicks > 0) {
    if (pcnt_set_filter_value(_unit, filterTicks) != ESP_OK) return false;
    if (pcnt_filter_enable(_unit) != ESP_OK) return false;
  } else {
    // IMPORTANTÍSIMO: si es 0, NO habilitar filtro
    pcnt_filter_disable(_unit);
  }

  if (pcnt_counter_pause(_unit) != ESP_OK) return false;
  if (pcnt_counter_clear(_unit) != ESP_OK) return false;

  // evento al llegar al límite alto
  if (pcnt_event_enable(_unit, PCNT_EVT_H_LIM) != ESP_OK) return false;

  if (!_isrServiceInstalled) {
    if (pcnt_isr_service_install(0) != ESP_OK) return false;
    _isrServiceInstalled = true;
  }

  // Por las dudas, si ya estaba agregado, lo removemos (evita duplicados al reiniciar)
  pcnt_isr_handler_remove(_unit);

  if (pcnt_isr_handler_add(_unit, pcntIsr, (void*)_unit) != ESP_OK) return false;

  _ovf[_unit] = 0;

  if (pcnt_counter_resume(_unit) != ESP_OK) return false;
  return true;
}

void IRAM_ATTR RTC32K_Pcnt_Module::pcntIsr(void* arg) {
  pcnt_unit_t unit = (pcnt_unit_t)(uint32_t)arg;

  uint32_t st = 0;
  pcnt_get_event_status(unit, &st);

  if (st & PCNT_EVT_H_LIM) {
    _ovf[unit]++;             // overflow virtual
    pcnt_counter_clear(unit); // seguir contando (sí, esto mete un bias fijo de unos pulsos)
  }
}

uint32_t RTC32K_Pcnt_Module::readAndReset() {
  // Pausamos para leer consistente
  pcnt_counter_pause(_unit);

  int16_t c = 0;
  pcnt_get_counter_value(_unit, &c);

  noInterrupts();
  uint32_t ov = _ovf[_unit];
  _ovf[_unit] = 0;
  interrupts();

  pcnt_counter_clear(_unit);
  pcnt_counter_resume(_unit);

  // Cada overflow equivale a 32768 pulsos (porque limpiamos al llegar a 32767)
  return ov * 32768UL + (uint16_t)c;
}

uint32_t RTC32K_Pcnt_Module::getOverflowCount() {
  noInterrupts();
  uint32_t v = _ovf[_unit];
  interrupts();
  return v;
}