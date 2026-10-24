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

    // Sólo inicia UART/NMEA. Usar este método cuando el PPS lo maneja otra parte
    // del sistema, por ejemplo el main junto con PCNT.
    void beginNmeaOnly();

    // Inicia UART/NMEA y además engancha la ISR PPS interna del módulo GPS.
    // No usar si el PPS ya se usa en otra ISR del sistema.
    void begin();

    void update();

    // RMC válida
    bool hasValidRmcRecent(unsigned long maxAgeMs) const;
    bool getLastValidLocalDateTime(DateTime& outLocal) const;
    bool hasLastValidRmc() const;
    unsigned long getLastValidRmcAgeMs() const;

    // GNSS “alineado al PPS” usando el contador PPS interno de este módulo.
    // Sólo tiene sentido si se llamó begin(), no beginNmeaOnly().
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

    // PPS interno del GPS_Module
    unsigned long getPpsPulseCount() const;
    unsigned long getPpsPeriodUs() const;
    bool hasSeenPps() const;
    unsigned long getLastPpsUs() const;

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

    // Base estable GNSS@PPS (sólo para PPS interno del GPS_Module)
    DateTime _lastRmcLocal = DateTime(2000,1,1,0,0,0);
    bool _haveLastRmcLocal = false;
    unsigned long _ppsCountAtLastRmc = 0;

    // Contadores
    unsigned long _cntTXT   = 0;
    unsigned long _cntRMC   = 0;
    unsigned long _cntGGA   = 0;
    unsigned long _cntGSA   = 0;
    unsigned long _cntOTROS = 0;

    // PPS interno (static para ISR)
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