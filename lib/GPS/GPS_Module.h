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
    GPS_Module(HardwareSerial& serial, int rxPin, int txPin, int ppsPin, long baud = 9600);

    void begin();
    void update();

    // RMC válida
    bool hasValidRmcRecent(unsigned long maxAgeMs) const;
    bool getLastValidLocalDateTime(DateTime& outLocal) const;
    bool hasLastValidRmc() const;
    unsigned long getLastValidRmcAgeMs() const;

    // Contadores NMEA
    unsigned long getTxtCount() const;
    unsigned long getRmcCount() const;
    unsigned long getGgaCount() const;
    unsigned long getOtherCount() const;

    // PPS
    unsigned long getPpsPulseCount() const;
    unsigned long getPpsPeriodUs() const;
    bool hasSeenPps() const;

private:
    HardwareSerial& _serial;
    int _rxPin;
    int _txPin;
    int _ppsPin;
    long _baud;

    // NMEA
    String _nmeaLine;
    bool _inSentence = false;
    unsigned long _lastByteMs = 0;

    // Última RMC válida
    RmcData _lastValidRmc;
    bool _haveLastValidRmc = false;
    unsigned long _lastValidRmcMs = 0;

    // Contadores
    unsigned long _cntTXT = 0;
    unsigned long _cntRMC = 0;
    unsigned long _cntGGA = 0;
    unsigned long _cntOTROS = 0;

    // PPS (static para ISR)
    static volatile unsigned long _ppsPulses;
    static volatile unsigned long _ppsLastUs;
    static volatile unsigned long _ppsPeriodUs;
    static volatile bool _ppsSeen;

    static void IRAM_ATTR ppsISR();

    void processSentence(const String& line);
    void closeSentenceIfNeeded();

    bool parseRMC(const String& s, RmcData& out);
    bool rmcEsRazonable(const RmcData& r) const;
    bool rmcToLocalDateTimeUTCminus3(const RmcData& r, DateTime& outLocal) const;
};

#endif