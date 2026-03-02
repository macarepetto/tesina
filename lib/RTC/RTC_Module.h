#ifndef RTC_MODULE_H
#define RTC_MODULE_H

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>

class RTC_Module {
public:
    RTC_Module(int sdaPin, int sclPin, int sqwPin);

    bool begin();
    void adjustToCompileTime();
    void adjust(const DateTime& dt);
    DateTime now();

    unsigned long getSqwPulses();
    int getSqwPin() const;

private:
    int _sdaPin;
    int _sclPin;
    int _sqwPin;

    RTC_DS3231 _rtc;

    static volatile unsigned long _sqwPulses;

    static void IRAM_ATTR sqwISR();
};

#endif