#ifndef RTC_MODULE_H
#define RTC_MODULE_H

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>

class RTC_Module {
public:
    // sdaPin/sclPin en -1 usa los pines I2C por defecto de la placa.
    // sqwPin en -1 no configura la salida SQW ni su interrupción.
    RTC_Module(int sdaPin = -1, int sclPin = -1, int sqwPin = -1);

    bool begin();

    void adjustToCompileTime();
    void adjust(const DateTime& dt);
    DateTime now();

    // Escritura/lectura explícita de fecha/hora DS3231.
    bool setFromDateTime(const DateTime& dt);
    bool readDateTime(DateTime& dt);
    bool readEpoch(int64_t& epoch_s);

    // Aging DS3231.
    bool readAging(int8_t& value);
    bool applyAgingValue(int8_t value);
    int8_t getAppliedAging() const;

    // Formato ISO local sin zona horaria, usado por logs.
    static void dateTimeToIsoLocal(const DateTime& dt, char* out, size_t len);
    static void epochToIsoLocal(int64_t epoch_s, char* out, size_t len);

    // SQW opcional.
    unsigned long getSqwPulses();
    int getSqwPin() const;
    unsigned long getLastSqwUs() const;

private:
    int _sdaPin;
    int _sclPin;
    int _sqwPin;
    int8_t _agingApplied = 0;

    RTC_DS3231 _rtc;

    static constexpr uint8_t DS3231_ADDRESS = 0x68;
    static constexpr uint8_t REG_CONTROL = 0x0E;
    static constexpr uint8_t REG_STATUS  = 0x0F;
    static constexpr uint8_t REG_AGING   = 0x10;
    static constexpr uint8_t BIT_CONV = 5;
    static constexpr uint8_t BIT_BSY  = 2;

    static uint8_t decToBcd(uint8_t value);
    static uint8_t bcdToDec(uint8_t value);

    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);
    bool waitNotBusy(uint32_t timeout_ms);
    bool forceAgingUpdate();

    static bool dateTimeIsReasonable(const DateTime& dt);

    static volatile unsigned long _sqwPulses;
    static volatile unsigned long _sqwLastUs;

    static void IRAM_ATTR sqwISR();
};

#endif