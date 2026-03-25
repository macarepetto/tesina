#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>
#include <RTClib.h>

struct RmcData {
    bool valid = false;     // A/V
    int hour = 0, minute = 0, second = 0;
    int day = 0, month = 0, year = 0; // full year
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

    // GNSS “alineado al PPS” (estable, NO depende del WiFi)
    // Devuelve la hora local (UTC-3) estimada para el "segundo PPS" actual
    bool getGpsLocalAtPps(DateTime& outLocal) const;

    // Contadores NMEA
    unsigned long getTxtCount() const;
    unsigned long getRmcCount() const;
    unsigned long getGgaCount() const;
    unsigned long getGsaCount() const;
    unsigned long getOtherCount() const;

    // Métricas GNSS
    int   getGgaNumSats() const;
    float getGgaHdop() const;

    int   getGsaFixType() const;   // 1=no fix, 2=2D, 3=3D
    float getGsaPdop() const;
    float getGsaHdop() const;
    float getGsaVdop() const;

    // PPS “limpio”
    unsigned long getPpsPulseCount() const; // pulsos PPS aceptados como PPS real
    unsigned long getPpsPeriodUs() const;   // último período aceptado (us)
    bool hasSeenPps() const;                // vio PPS aceptado alguna vez
    unsigned long getLastPpsUs() const;     // micros() del último PPS aceptado

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

    // Métricas parseadas
    int   _ggaNumSats = -1;
    float _ggaHdop    = -1.0f;

    int   _gsaFixType = -1;
    float _gsaPdop    = -1.0f;
    float _gsaHdop    = -1.0f;
    float _gsaVdop    = -1.0f;

    // Última RMC válida
    RmcData _lastValidRmc;
    bool _haveLastValidRmc = false;
    unsigned long _lastValidRmcMs = 0;

    // Base estable GNSS@PPS (se actualiza al recibir RMC válida)
    DateTime _lastRmcLocal = DateTime(2000,1,1,0,0,0);
    bool _haveLastRmcLocal = false;
    unsigned long _ppsCountAtLastRmc = 0;

    // Contadores
    unsigned long _cntTXT   = 0;
    unsigned long _cntRMC   = 0;
    unsigned long _cntGGA   = 0;
    unsigned long _cntGSA   = 0;
    unsigned long _cntOTROS = 0;

    // PPS (static para ISR)
    static volatile unsigned long _ppsPulses;
    static volatile unsigned long _ppsLastUs;
    static volatile unsigned long _ppsPeriodUs;
    static volatile bool          _ppsSeen;
    static volatile unsigned long _ppsLastUsValid;

    static void IRAM_ATTR ppsISR();

    void processSentence(const String& line);
    void closeSentenceIfNeeded();

    bool parseRMC(const String& s, RmcData& out);
    bool rmcEsRazonable(const RmcData& r) const;
    bool rmcToLocalDateTimeUTCminus3(const RmcData& r, DateTime& outLocal) const;
};

#endif