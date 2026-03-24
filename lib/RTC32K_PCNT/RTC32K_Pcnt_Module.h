#ifndef RTC32K_PCNT_MODULE_H
#define RTC32K_PCNT_MODULE_H

#include <Arduino.h>
#include "driver/pcnt.h"

class RTC32K_Pcnt_Module {
public:
  RTC32K_Pcnt_Module(int pin32k,
                     pcnt_unit_t unit = PCNT_UNIT_0,
                     pcnt_channel_t ch = PCNT_CHANNEL_0);

  bool begin(uint16_t filterTicks = 0);
  uint32_t readAndReset();
  uint32_t getOverflowCount();

private:
  int _pin;
  pcnt_unit_t _unit;
  pcnt_channel_t _ch;

  static volatile uint32_t _ovf[PCNT_UNIT_MAX];
  static bool _isrServiceInstalled;

  static void IRAM_ATTR pcntIsr(void* arg);
};

#endif