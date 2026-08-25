#include "RTC32K_Pcnt_Module.h"

volatile uint64_t RTC32K_Pcnt_Module::_acc[PCNT_UNIT_MAX] = {0};
bool RTC32K_Pcnt_Module::_isrServiceInstalled = false;

RTC32K_Pcnt_Module::RTC32K_Pcnt_Module(int pin32k, pcnt_unit_t unit, pcnt_channel_t ch)
  : _pin(pin32k), _unit(unit), _ch(ch) {}

bool RTC32K_Pcnt_Module::begin(uint16_t filterTicks) {
  // Muchas salidas 32K de DS3231 (según módulo) pueden necesitar pull-up.
  pinMode(_pin, INPUT_PULLUP);

  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = _pin;
  cfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
  cfg.unit           = _unit;
  cfg.channel        = _ch;

  // Más robusto: contar flanco de bajada (falling). La bajada suele ser “fuerte”.
  cfg.pos_mode = PCNT_COUNT_DIS;
  cfg.neg_mode = PCNT_COUNT_INC;

  cfg.lctrl_mode = PCNT_MODE_KEEP;
  cfg.hctrl_mode = PCNT_MODE_KEEP;

  cfg.counter_h_lim = 32767;
  cfg.counter_l_lim = 0;

  if (pcnt_unit_config(&cfg) != ESP_OK) return false;

  if (filterTicks > 0) {
    if (pcnt_set_filter_value(_unit, filterTicks) != ESP_OK) return false;
    if (pcnt_filter_enable(_unit) != ESP_OK) return false;
  } else {
    pcnt_filter_disable(_unit);
  }

  // Evento THRES1 para “flush” del contador a 64-bit
  if (pcnt_set_event_value(_unit, PCNT_EVT_THRES_1, CHUNK) != ESP_OK) return false;
  if (pcnt_event_enable(_unit, PCNT_EVT_THRES_1) != ESP_OK) return false;

  // Safety por si algo se va de rango
  pcnt_event_enable(_unit, PCNT_EVT_H_LIM);

  pcnt_counter_pause(_unit);
  pcnt_counter_clear(_unit);

  if (!_isrServiceInstalled) {
    if (pcnt_isr_service_install(0) != ESP_OK) return false;
    _isrServiceInstalled = true;
  }

  // Evitar duplicar handler si reiniciás
  pcnt_isr_handler_remove(_unit);
  if (pcnt_isr_handler_add(_unit, pcntISR, (void*)_unit) != ESP_OK) return false;

  _acc[_unit] = 0;
  pcnt_counter_resume(_unit);
  return true;
}

// Compatibilidad con variantes de IDF: a veces st trae bitmask (1<<evt), a veces evt ya es máscara.
bool RTC32K_Pcnt_Module::evtHit(uint32_t st, pcnt_evt_type_t evt) {
  const uint32_t e = (uint32_t)evt;
  if (st & e) return true;
  if (e < 32 && (st & (1UL << e))) return true;
  return false;
}

void IRAM_ATTR RTC32K_Pcnt_Module::pcntISR(void* arg) {
  const pcnt_unit_t unit = (pcnt_unit_t)(uint32_t)arg;

  uint32_t st = 0;
  pcnt_get_event_status(unit, &st);

  if (evtHit(st, PCNT_EVT_THRES_1) || evtHit(st, PCNT_EVT_H_LIM)) {
    int16_t c = 0;
    pcnt_get_counter_value(unit, &c);

    _acc[unit] += (uint16_t)c;  // “flush”
    pcnt_counter_clear(unit);   // seguir contando desde 0
  }
}

uint64_t RTC32K_Pcnt_Module::read_counter() const {
  // Lectura consistente: acc + contador actual
  noInterrupts();
  uint64_t acc = _acc[_unit];
  int16_t c = 0;
  pcnt_get_counter_value(_unit, &c);
  interrupts();

  return acc + (uint16_t)c;
}