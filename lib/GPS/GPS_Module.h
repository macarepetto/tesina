#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>
#include <RTClib.h>

struct RmcData {
    bool valid = false;     // A/V
    int hour = 0, minute = 0, second = 0;
    int day = 0, month = 0, year = 0;
};

class GPS_Module {
public:
    GPS_Module(HardwareSerial& serial, int rxPin, int txPin, long baud = 9600);

    void begin();
    void update();

    bool hasValidRmcRecent(unsigned long maxAgeMs) const;
    bool getLastValidLocalDateTime(DateTime& outLocal) const;

    unsigned long getTxtCount() const;
    unsigned long getRmcCount() const;
    unsigned long getGgaCount() const;
    unsigned long getOtherCount() const;

    bool hasLastValidRmc() const;
    unsigned long getLastValidRmcAgeMs() const;

private:
    HardwareSerial& _serial;
    int _rxPin;
    int _txPin;
    long _baud;

    String _nmeaLine;
    bool _inSentence = false;
    unsigned long _lastByteMs = 0;

    RmcData _lastValidRmc;
    bool _haveLastValidRmc = false;
    unsigned long _lastValidRmcMs = 0;

    unsigned long _cntTXT = 0;
    unsigned long _cntRMC = 0;
    unsigned long _cntGGA = 0;
    unsigned long _cntOTROS = 0;

    void processSentence(const String& line);
    void closeSentenceIfNeeded();

    bool parseRMC(const String& s, RmcData& out);
    bool rmcEsRazonable(const RmcData& r) const;
    bool rmcToLocalDateTimeUTCminus3(const RmcData& r, DateTime& outLocal) const;
};

#endif