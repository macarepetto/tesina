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

  // límite 16-bit signed: 32767. OJO: 32kHz * 1s = 32768, justo 1 más.
  cfg.counter_h_lim = 32767;
  cfg.counter_l_lim = 0;

  if (pcnt_unit_config(&cfg) != ESP_OK) return false;

  // Filtro anti-glitch (en ticks de APB). 400 ~ 5us aprox (depende del core).
  pcnt_set_filter_value(_unit, filterTicks);
  pcnt_filter_enable(_unit);

  pcnt_counter_pause(_unit);
  pcnt_counter_clear(_unit);

  // evento por llegar al límite alto
  pcnt_event_enable(_unit, PCNT_EVT_H_LIM);

  if (!_isrServiceInstalled) {
    // instala servicio ISR una sola vez
    pcnt_isr_service_install(0);
    _isrServiceInstalled = true;
  }

  pcnt_isr_handler_add(_unit, pcntIsr, (void*)_unit);

  _ovf[_unit] = 0;
  pcnt_counter_resume(_unit);
  return true;
}

void IRAM_ATTR RTC32K_Pcnt_Module::pcntIsr(void* arg) {
  pcnt_unit_t unit = (pcnt_unit_t)(uint32_t)arg;

  uint32_t st = 0;
  pcnt_get_event_status(unit, &st);

  // En este SDK, st es un bitmask con eventos (PCNT_EVT_H_LIM, etc.)
  if (st & PCNT_EVT_H_LIM) {
    _ovf[unit]++;             // overflow “virtual”
    pcnt_counter_clear(unit); // volvemos a 0 para seguir contando
  }
}

uint32_t RTC32K_Pcnt_Module::readAndReset() {
  // Pausamos para leer consistente y resetear todo junto
  pcnt_counter_pause(_unit);

  int16_t c = 0;
  pcnt_get_counter_value(_unit, &c);

  noInterrupts();
  uint32_t ov = _ovf[_unit];
  _ovf[_unit] = 0;
  interrupts();

  pcnt_counter_clear(_unit);
  pcnt_counter_resume(_unit);

  // Cada overflow equivale a 32768 pulsos (porque al llegar a 32767 limpiamos)
  return ov * 32768UL + (uint16_t)c;
}

uint32_t RTC32K_Pcnt_Module::getOverflowCount() {
  noInterrupts();
  uint32_t v = _ovf[_unit];
  interrupts();
  return v;
}