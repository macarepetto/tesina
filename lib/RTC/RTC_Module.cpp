#include "RTC_Module.h"

volatile unsigned long RTC_Module::_sqwPulses = 0;
volatile unsigned long RTC_Module::_sqwLastUs = 0;

RTC_Module::RTC_Module(int sdaPin, int sclPin, int sqwPin)
    : _sdaPin(sdaPin), _sclPin(sclPin), _sqwPin(sqwPin) {}

bool RTC_Module::begin() {
    if (_sdaPin >= 0 && _sclPin >= 0) {
        Wire.begin(_sdaPin, _sclPin);
    } else {
        Wire.begin();
    }

    if (!_rtc.begin()) {
        return false;
    }

    if (_sqwPin >= 0) {
        _rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
        pinMode(_sqwPin, INPUT_PULLUP);
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

uint8_t RTC_Module::decToBcd(uint8_t value) {
    return ((value / 10) << 4) | (value % 10);
}

uint8_t RTC_Module::bcdToDec(uint8_t value) {
    return ((value >> 4) * 10) + (value & 0x0F);
}

bool RTC_Module::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool RTC_Module::readRegister(uint8_t reg, uint8_t& value) {
    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(DS3231_ADDRESS, (uint8_t)1) != 1) {
        return false;
    }

    value = Wire.read();
    return true;
}

bool RTC_Module::waitNotBusy(uint32_t timeout_ms) {
    const uint32_t inicio = millis();

    while (millis() - inicio < timeout_ms) {
        uint8_t status = 0;
        if (!readRegister(REG_STATUS, status)) {
            return false;
        }

        if ((status & (1U << BIT_BSY)) == 0) {
            return true;
        }

        delay(5);
    }

    return false;
}

bool RTC_Module::forceAgingUpdate() {
    if (!waitNotBusy(500)) {
        return false;
    }

    uint8_t control = 0;
    if (!readRegister(REG_CONTROL, control)) {
        return false;
    }

    control |= (1U << BIT_CONV);
    if (!writeRegister(REG_CONTROL, control)) {
        return false;
    }

    const uint32_t inicio = millis();
    while (millis() - inicio < 1000) {
        if (!readRegister(REG_CONTROL, control)) {
            return false;
        }

        if ((control & (1U << BIT_CONV)) == 0) {
            return waitNotBusy(500);
        }

        delay(5);
    }

    return false;
}

bool RTC_Module::readAging(int8_t& value) {
    uint8_t raw = 0;
    if (!readRegister(REG_AGING, raw)) {
        return false;
    }

    value = (int8_t)raw;
    return true;
}

bool RTC_Module::applyAgingValue(int8_t value) {
    if (!writeRegister(REG_AGING, (uint8_t)value)) {
        return false;
    }

    if (!forceAgingUpdate()) {
        return false;
    }

    int8_t readback = 0;
    if (!readAging(readback)) {
        return false;
    }

    _agingApplied = readback;
    return _agingApplied == value;
}

int8_t RTC_Module::getAppliedAging() const {
    return _agingApplied;
}

bool RTC_Module::dateTimeIsReasonable(const DateTime& dt) {
    if (dt.year() < 2020 || dt.year() > 2099) return false;
    if (dt.month() < 1 || dt.month() > 12) return false;
    if (dt.day() < 1 || dt.day() > 31) return false;
    if (dt.hour() > 23) return false;
    if (dt.minute() > 59) return false;
    if (dt.second() > 59) return false;
    return true;
}

bool RTC_Module::setFromDateTime(const DateTime& dt) {
    if (dt.year() < 2000 || dt.year() > 2099) {
        return false;
    }

    const uint8_t year_2_digits = (uint8_t)(dt.year() - 2000);
    const uint8_t day_of_week = (uint8_t)(dt.dayOfTheWeek() + 1); // RTClib: 0=domingo

    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write((uint8_t)0x00);                 // Registro segundos
    Wire.write(decToBcd((uint8_t)dt.second()));
    Wire.write(decToBcd((uint8_t)dt.minute()));
    Wire.write(decToBcd((uint8_t)dt.hour()));  // 24 h
    Wire.write(decToBcd(day_of_week));
    Wire.write(decToBcd((uint8_t)dt.day()));
    Wire.write(decToBcd((uint8_t)dt.month()));
    Wire.write(decToBcd(year_2_digits));

    return Wire.endTransmission() == 0;
}

bool RTC_Module::readDateTime(DateTime& dt) {
    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write((uint8_t)0x00);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(DS3231_ADDRESS, (uint8_t)7) != 7) {
        return false;
    }

    uint8_t raw_sec = Wire.read();
    uint8_t raw_min = Wire.read();
    uint8_t raw_hour = Wire.read();
    Wire.read(); // día de semana; no se usa para el cálculo
    uint8_t raw_day = Wire.read();
    uint8_t raw_month = Wire.read();
    uint8_t raw_year = Wire.read();

    const uint8_t second = bcdToDec(raw_sec & 0x7F);
    const uint8_t minute = bcdToDec(raw_min & 0x7F);
    const uint8_t hour   = bcdToDec(raw_hour & 0x3F);
    const uint8_t day    = bcdToDec(raw_day & 0x3F);
    const uint8_t month  = bcdToDec(raw_month & 0x1F);
    const uint16_t year  = 2000 + bcdToDec(raw_year);

    if (
        year < 2020 || year > 2099 ||
        month < 1 || month > 12 ||
        day < 1 || day > 31 ||
        hour > 23 ||
        minute > 59 ||
        second > 59
    ) {
        return false;
    }

    dt = DateTime(year, month, day, hour, minute, second);
    return true;
}

bool RTC_Module::readEpoch(int64_t& epoch_s) {
    DateTime dt(2000, 1, 1, 0, 0, 0);
    if (!readDateTime(dt)) {
        return false;
    }

    epoch_s = (int64_t)dt.unixtime();
    return true;
}

void RTC_Module::dateTimeToIsoLocal(const DateTime& dt, char* out, size_t len) {
    snprintf(
        out,
        len,
        "%04d-%02d-%02dT%02d:%02d:%02d",
        dt.year(),
        dt.month(),
        dt.day(),
        dt.hour(),
        dt.minute(),
        dt.second()
    );
}

void RTC_Module::epochToIsoLocal(int64_t epoch_s, char* out, size_t len) {
    if (epoch_s < 0 || epoch_s > 0xFFFFFFFFLL) {
        snprintf(out, len, "null");
        return;
    }

    DateTime dt((uint32_t)epoch_s);
    dateTimeToIsoLocal(dt, out, len);
}

unsigned long RTC_Module::getSqwPulses() {
    noInterrupts();
    unsigned long value = _sqwPulses;
    interrupts();
    return value;
}

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
    _sqwLastUs = micros();
    _sqwPulses++;
}