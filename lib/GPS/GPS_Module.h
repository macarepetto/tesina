#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>
#include <RTClib.h>

struct RmcData {
    bool valid = false;     // A/V
    int hour = 0, minute = 0, second = 0;
    int day = 0, month = 0, year = 0; // full year
};

// Diagnostico observacional de RMC: no valida la fase UTC del PPS.
struct RmcDiagnostics {
    uint32_t validas = 0;
    uint32_t checksum_fallidos = 0;
    uint32_t rechazadas = 0;
    uint32_t saltos = 0;
    uint32_t revision = 0;
    uint32_t procesado_us = 0; // micros() al terminar de procesar RMC; NO recepcion fisica.
    uint32_t intervalo_ms = 0;
    int32_t ultimo_delta_s = 0;
    int32_t ultimo_error_continuidad_s = 0;
};

// Consulta UBX-NAV-TIMEUTC: observacional; no certifica fase absoluta del PPS.
struct UbxUtcDiagnostics {
    uint32_t consultas = 0;
    uint32_t respuestas = 0;
    uint32_t checksum_fallos = 0;
    bool recibido = false;
    bool fecha_razonable = false;
    uint8_t flags = 0;
    uint32_t iTOW_ms = 0;
    uint32_t tAcc_ns = 0;
    int32_t nano_ns = 0;
    uint16_t year = 0;
    uint8_t month = 0, day = 0, hour = 0, minute = 0, second = 0;
    uint32_t recibido_ms = 0;
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

    // Poll liviano de UTC por UART existente. NO configura salida UBX continua.
    // Requiere conexion fisica ESP32 TX -> RX del modulo GNSS.
    void consultarUbxUtcSiCorresponde();
    UbxUtcDiagnostics getUbxUtcDiagnostics() const;

    // RMC válida
    bool hasValidRmcRecent(unsigned long maxAgeMs) const;
    bool getLastValidLocalDateTime(DateTime& outLocal) const;
    bool hasLastValidRmc() const;
    unsigned long getLastValidRmcAgeMs() const;
    RmcDiagnostics getRmcDiagnostics() const;

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
    RmcDiagnostics _rmcDiag;
    UbxUtcDiagnostics _ubxUtc;

    // Parser binario acotado al mensaje UBX-NAV-TIMEUTC (20 bytes).
    uint8_t _ubxState = 0;
    uint8_t _ubxClass = 0, _ubxId = 0;
    uint8_t _ubxCkA = 0, _ubxCkB = 0;
    uint16_t _ubxLength = 0, _ubxIndex = 0;
    uint8_t _ubxPayload[64] = {};
    uint32_t _ubxLastPollMs = 0;
    bool consumirByteUbx(uint8_t byte);
    void incluirEnChecksumUbx(uint8_t byte);
    void interpretarUbxTimeUtc();

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