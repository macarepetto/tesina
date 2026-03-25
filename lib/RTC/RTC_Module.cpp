#include "RTC_Module.h"

volatile unsigned long RTC_Module::_sqwPulses = 0;
volatile unsigned long RTC_Module::_sqwLastUs = 0; // NUEVA INICIALIZACIÓN

RTC_Module::RTC_Module(int sdaPin, int sclPin, int sqwPin)
    : _sdaPin(sdaPin), _sclPin(sclPin), _sqwPin(sqwPin) {}

bool RTC_Module::begin() {
    Wire.begin(_sdaPin, _sclPin);

    if (!_rtc.begin()) {
        return false;
    }

    if (_sqwPin >= 0) {
        // 1. Encendemos la onda cuadrada a 1Hz en el DS3231
        _rtc.writeSqwPinMode(DS3231_SquareWave1Hz);

        // 2. Configuramos el pin del ESP32 con PULLUP
        pinMode(_sqwPin, INPUT_PULLUP); 
        
        // 3. El inicio del segundo en el DS3231 ocurre en el flanco de BAJADA
        attachInterrupt(digitalPinToInterrupt(_sqwPin), sqwISR, FALLING); 
    }

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

// NUEVO: Función que devuelve el timestamp de forma segura
unsigned long RTC_Module::getLastSqwUs() const {
    noInterrupts();
    unsigned long value = _sqwLastUs;
    interrupts();
    return value;
}

int RTC_Module::getSqwPin() const {
    return _sqwPin;
}

void IRAM_ATTR RTC_Module::sqwISR() {
    _sqwLastUs = micros(); // NUEVO: Capturamos el tiempo exacto
    _sqwPulses++;
}