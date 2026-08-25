#ifndef RTC32K_PCNT_MODULE_H
#define RTC32K_PCNT_MODULE_H

#include <Arduino.h>
#include "driver/pcnt.h"

// Cuenta la señal 32kHz del DS3231 usando PCNT y expone un contador continuo 64-bit.
// No resetea tu medición: "flushea" el 16-bit interno a un acumulador 64-bit.
class RTC32K_Pcnt_Module {
public:
  explicit RTC32K_Pcnt_Module(int pin32k,
                              pcnt_unit_t unit = PCNT_UNIT_0,
                              pcnt_channel_t ch = PCNT_CHANNEL_0);

  // filterTicks: filtro anti-glitch del PCNT (ticks APB). Para 32kHz normalmente 0.
  bool begin(uint16_t filterTicks = 0);

  // Contador continuo (NO reset, NO pause).
  uint64_t read_counter() const;

private:
  int _pin;
  pcnt_unit_t _unit;
  pcnt_channel_t _ch;

  static constexpr int16_t CHUNK = 16000; // flush 2 veces por segundo aprox

  static volatile uint64_t _acc[PCNT_UNIT_MAX];
  static bool _isrServiceInstalled;

  static bool evtHit(uint32_t st, pcnt_evt_type_t evt);
  static void IRAM_ATTR pcntISR(void* arg);
};

#endif