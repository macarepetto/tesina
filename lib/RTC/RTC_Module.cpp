#include "RTC_Module.h"

volatile unsigned long RTC_Module::_sqwPulses = 0;

RTC_Module::RTC_Module(int sdaPin, int sclPin, int sqwPin)
    : _sdaPin(sdaPin), _sclPin(sclPin), _sqwPin(sqwPin) {}

bool RTC_Module::begin() {
    Wire.begin(_sdaPin, _sclPin);

    if (!_rtc.begin()) {
        return false;
    }

    _rtc.writeSqwPinMode(DS3231_SquareWave1Hz);

    pinMode(_sqwPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_sqwPin), sqwISR, RISING);

    return true;
}

void RTC_Module::adjustToCompileTime() {
    DateTime t(__DATE__, __TIME__);
    _rtc.adjust(t);
}

void RTC_Module::adjust(const DateTime& dt) {
    _rtc.adjust(dt);
}

DateTime RTC_Module::now() {
    return _rtc.now();
}

unsigned long RTC_Module::getSqwPulses() {
    noInterrupts();
    unsigned long value = _sqwPulses;
    interrupts();
    return value;
}

int RTC_Module::getSqwPin() const {
    return _sqwPin;
}

void IRAM_ATTR RTC_Module::sqwISR() {
    _sqwPulses++;
}