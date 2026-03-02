#include "GPS_Module.h"

GPS_Module::GPS_Module(HardwareSerial& serial, int rxPin, int txPin, long baud)
    : _serial(serial), _rxPin(rxPin), _txPin(txPin), _baud(baud) {}

void GPS_Module::begin() {
    _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
}

void GPS_Module::update() {
    while (_serial.available()) {
        char c = (char)_serial.read();
        _lastByteMs = millis();

        if (c == '$') {
            closeSentenceIfNeeded();
            _inSentence = true;
            _nmeaLine = "$";
            continue;
        }

        if (!_inSentence) continue;

        if (c == '\r' || c == '\n') {
            closeSentenceIfNeeded();
            continue;
        }

        if (_nmeaLine.length() < 200) {
            _nmeaLine += c;
        } else {
            _inSentence = false;
            _nmeaLine = "";
        }
    }

    if (_inSentence && (millis() - _lastByteMs) > 200) {
        closeSentenceIfNeeded();
    }
}

void GPS_Module::closeSentenceIfNeeded() {
    if (_inSentence && _nmeaLine.length() > 6) {
        processSentence(_nmeaLine);
    }
    _inSentence = false;
    _nmeaLine = "";
}

void GPS_Module::processSentence(const String& line) {
    if (line.length() < 6) return;

    if (line.startsWith("$GPTXT") || line.startsWith("$GNTXT")) _cntTXT++;
    else if (line.startsWith("$GPRMC") || line.startsWith("$GNRMC")) _cntRMC++;
    else if (line.startsWith("$GPGGA") || line.startsWith("$GNGGA")) _cntGGA++;
    else _cntOTROS++;

    RmcData r;
    if (parseRMC(line, r) && rmcEsRazonable(r)) {
        _lastValidRmc = r;
        _haveLastValidRmc = true;
        _lastValidRmcMs = millis();
    }
}

bool GPS_Module::parseRMC(const String& s, RmcData& out) {
    if (!(s.startsWith("$GPRMC") || s.startsWith("$GNRMC"))) return false;

    int commas[32];
    int n = 0;
    for (int i = 0; i < (int)s.length() && n < 32; i++) {
        if (s[i] == ',') commas[n++] = i;
    }
    if (n < 9) return true;

    auto field = [&](int idx)->String {
        int start = (idx == 0) ? 0 : commas[idx - 1] + 1;
        int end   = (idx < n) ? commas[idx] : s.length();
        if (start >= (int)s.length() || end > (int)s.length() || start > end) return "";
        return s.substring(start, end);
    };

    String timeStr = field(1);
    String status  = field(2);
    String dateStr = field(9);

    out.valid = (status.length() > 0 && status[0] == 'A');

    if (timeStr.length() >= 6) {
        out.hour   = timeStr.substring(0, 2).toInt();
        out.minute = timeStr.substring(2, 4).toInt();
        out.second = timeStr.substring(4, 6).toInt();
    }

    if (dateStr.length() >= 6) {
        out.day   = dateStr.substring(0, 2).toInt();
        out.month = dateStr.substring(2, 4).toInt();
        int yy    = dateStr.substring(4, 6).toInt();
        out.year  = 2000 + yy;
    }

    return true;
}

bool GPS_Module::rmcEsRazonable(const RmcData& r) const {
    if (!r.valid) return false;

    if (r.year < 2020 || r.year > 2100) return false;
    if (r.month < 1 || r.month > 12) return false;
    if (r.day < 1 || r.day > 31) return false;
    if (r.hour < 0 || r.hour > 23) return false;
    if (r.minute < 0 || r.minute > 59) return false;
    if (r.second < 0 || r.second > 59) return false;

    return true;
}

bool GPS_Module::rmcToLocalDateTimeUTCminus3(const RmcData& r, DateTime& outLocal) const {
    if (!rmcEsRazonable(r)) return false;

    DateTime utc(r.year, r.month, r.day, r.hour, r.minute, r.second);
    uint32_t t = utc.unixtime();
    if (t < 3UL * 3600UL) return false;

    t -= 3UL * 3600UL;
    outLocal = DateTime(t);
    return true;
}

bool GPS_Module::hasValidRmcRecent(unsigned long maxAgeMs) const {
    if (!_haveLastValidRmc) return false;
    return (millis() - _lastValidRmcMs) <= maxAgeMs;
}

bool GPS_Module::getLastValidLocalDateTime(DateTime& outLocal) const {
    if (!_haveLastValidRmc) return false;
    return rmcToLocalDateTimeUTCminus3(_lastValidRmc, outLocal);
}

unsigned long GPS_Module::getTxtCount() const { return _cntTXT; }
unsigned long GPS_Module::getRmcCount() const { return _cntRMC; }
unsigned long GPS_Module::getGgaCount() const { return _cntGGA; }
unsigned long GPS_Module::getOtherCount() const { return _cntOTROS; }

bool GPS_Module::hasLastValidRmc() const { return _haveLastValidRmc; }

unsigned long GPS_Module::getLastValidRmcAgeMs() const {
    if (!_haveLastValidRmc) return 0;
    return millis() - _lastValidRmcMs;
}