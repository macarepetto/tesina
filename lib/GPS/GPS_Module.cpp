#include "GPS_Module.h"

// ========= Static PPS =========
volatile unsigned long GPS_Module::_ppsPulses      = 0;
volatile unsigned long GPS_Module::_ppsLastUs      = 0;
volatile unsigned long GPS_Module::_ppsPeriodUs    = 0;
volatile bool          GPS_Module::_ppsSeen        = false;
volatile unsigned long GPS_Module::_ppsLastUsValid = 0;

GPS_Module::GPS_Module(HardwareSerial& serial, int rxPin, int txPin, int ppsPin, long baud)
    : _serial(serial), _rxPin(rxPin), _txPin(txPin), _ppsPin(ppsPin), _baud(baud) {}

void GPS_Module::beginNmeaOnly() {
    _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
}

void GPS_Module::begin() {
    beginNmeaOnly();

    // Si estás usando GPIO27/32 para poder pullup interno, esto está perfecto.
    // OJO: PPS normalmente es push-pull, el pullup interno no molesta.
    pinMode(_ppsPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_ppsPin), ppsISR, RISING);
}

void GPS_Module::update() {
    while (_serial.available()) {
        const uint8_t byte = (uint8_t)_serial.read();
        _lastByteMs = millis();
        // Una sentencia NMEA sigue procesandose igual. Solo se apartan del
        // parser ASCII los bytes que forman una trama binaria UBX.
        if (consumirByteUbx(byte)) continue;
        char c = (char)byte;

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

    // timeout de sentencia incompleta
    if (_inSentence && (millis() - _lastByteMs) > 200) {
        closeSentenceIfNeeded();
    }
}


// UBX-NAV-TIMEUTC Poll: B5 62 / clase 01 / id 21 / longitud 0000 / CK 22 67.
// Consultamos cada 2s para NO modificar las salidas permanentes del NEO-6.
void GPS_Module::consultarUbxUtcSiCorresponde() {
    const uint32_t ahora = millis();
    if ((uint32_t)(ahora - _ubxLastPollMs) < 2000UL) return;
    _ubxLastPollMs = ahora;
    static const uint8_t poll[8] = {0xB5, 0x62, 0x01, 0x21,
                                    0x00, 0x00, 0x22, 0x67};
    if (_serial.write(poll, sizeof(poll)) == sizeof(poll)) {
        ++_ubxUtc.consultas;
    }
}

UbxUtcDiagnostics GPS_Module::getUbxUtcDiagnostics() const {
    return _ubxUtc;
}

void GPS_Module::incluirEnChecksumUbx(uint8_t b) {
    _ubxCkA = (uint8_t)(_ubxCkA + b);
    _ubxCkB = (uint8_t)(_ubxCkB + _ubxCkA);
}

static uint32_t ubxU4(const uint8_t* b) {
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

void GPS_Module::interpretarUbxTimeUtc() {
    if (_ubxClass != 0x01 || _ubxId != 0x21 || _ubxLength != 20) return;
    ++_ubxUtc.respuestas;
    _ubxUtc.recibido = true;
    _ubxUtc.recibido_ms = millis();
    _ubxUtc.iTOW_ms = ubxU4(&_ubxPayload[0]);
    _ubxUtc.tAcc_ns = ubxU4(&_ubxPayload[4]);
    _ubxUtc.nano_ns = (int32_t)ubxU4(&_ubxPayload[8]);
    _ubxUtc.year = (uint16_t)(_ubxPayload[12] | (_ubxPayload[13] << 8));
    _ubxUtc.month = _ubxPayload[14];
    _ubxUtc.day = _ubxPayload[15];
    _ubxUtc.hour = _ubxPayload[16];
    _ubxUtc.minute = _ubxPayload[17];
    _ubxUtc.second = _ubxPayload[18];
    _ubxUtc.flags = _ubxPayload[19];
    _ubxUtc.fecha_razonable =
        _ubxUtc.year >= 2020 && _ubxUtc.year <= 2099 &&
        _ubxUtc.month >= 1 && _ubxUtc.month <= 12 &&
        _ubxUtc.day >= 1 && _ubxUtc.day <= 31 &&
        _ubxUtc.hour <= 23 && _ubxUtc.minute <= 59 &&
        _ubxUtc.second <= 59;
}

// Retorna true cuando consumio el byte como parte de UBX.
// Permite intercalar sin mezclar UBX y NMEA en la misma UART.
bool GPS_Module::consumirByteUbx(uint8_t b) {
    switch (_ubxState) {
    case 0:
        if (b == 0xB5) { _ubxState = 1; return true; }
        return false;
    case 1:
        if (b == 0x62) {
            _ubxState = 2; _ubxCkA = 0; _ubxCkB = 0;
            _ubxLength = 0; _ubxIndex = 0;
            return true;
        }
        _ubxState = (b == 0xB5) ? 1 : 0;
        return b == 0xB5; // Si no era UBX, el byte vuelve a NMEA.
    case 2:
        _ubxClass = b; incluirEnChecksumUbx(b); _ubxState = 3; return true;
    case 3:
        _ubxId = b; incluirEnChecksumUbx(b); _ubxState = 4; return true;
    case 4:
        _ubxLength = b; incluirEnChecksumUbx(b); _ubxState = 5; return true;
    case 5:
        _ubxLength |= ((uint16_t)b << 8); incluirEnChecksumUbx(b);
        if (_ubxLength > sizeof(_ubxPayload)) {
            _ubxState = 0; // Descartar mensajes grandes; no los necesitamos.
        } else {
            _ubxState = _ubxLength ? 6 : 7;
        }
        return true;
    case 6:
        _ubxPayload[_ubxIndex++] = b;
        incluirEnChecksumUbx(b);
        if (_ubxIndex >= _ubxLength) _ubxState = 7;
        return true;
    case 7:
        if (b == _ubxCkA) _ubxState = 8;
        else {
            if (_ubxClass == 0x01 && _ubxId == 0x21)
                ++_ubxUtc.checksum_fallos;
            _ubxState = 0;
        }
        return true;
    case 8:
        if (b == _ubxCkB) interpretarUbxTimeUtc();
        else if (_ubxClass == 0x01 && _ubxId == 0x21)
            ++_ubxUtc.checksum_fallos;
        _ubxState = 0;
        return true;
    default:
        _ubxState = 0;
        return false;
    }
}

void GPS_Module::closeSentenceIfNeeded() {
    if (_inSentence && _nmeaLine.length() > 6) {
        processSentence(_nmeaLine);
    }
    _inSentence = false;
    _nmeaLine = "";
}

// ===== Helpers para parseo simple por comas =====
static String nmeaField(const String& s, int idx) {
    int field = 0;
    int start = 0;
    for (int i = 0; i <= (int)s.length(); i++) {
        if (i == (int)s.length() || s[i] == ',') {
            if (field == idx) return s.substring(start, i);
            field++;
            start = i + 1;
        }
    }
    return "";
}

static float toFloatSafe(String x) {
    if (x.length() == 0) return -1.0f;
    int star = x.indexOf('*');
    if (star >= 0) x = x.substring(0, star);
    return x.toFloat();
}

static int toIntSafe(String x) {
    if (x.length() == 0) return -1;
    int star = x.indexOf('*');
    if (star >= 0) x = x.substring(0, star);
    return x.toInt();
}

// NMEA-0183: XOR de los caracteres entre '$' y '*'.
// Se verifica solo RMC para mantener acotado este cambio experimental.
static int nmeaHex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static bool checksumRmcCorrecto(const String& s) {
    if (s.length() < 10 || s[0] != '$') return false;
    const int star = s.indexOf('*');
    if (star <= 1 || star + 3 != (int)s.length()) return false;
    const int hi = nmeaHex(s[star + 1]);
    const int lo = nmeaHex(s[star + 2]);
    if (hi < 0 || lo < 0) return false;
    uint8_t xorValue = 0;
    for (int i = 1; i < star; ++i) xorValue ^= (uint8_t)s[i];
    return xorValue == (uint8_t)((hi << 4) | lo);
}

void GPS_Module::processSentence(const String& line) {
    if (line.length() < 6) return;

    // Extraemos mágicamente las 3 letras del tipo de mensaje, ignorando si es GP, GN, GL, etc.
    String type = line.substring(3, 6); 

    // Contadores por tipo de mensaje real
    if (type == "TXT") _cntTXT++;
    else if (type == "RMC") _cntRMC++;
    else if (type == "GGA") _cntGGA++;
    else if (type == "GSA") _cntGSA++;
    else _cntOTROS++;

    // ---- Parse RMC (Cualquier constelación) ----
    RmcData r;
    if (type == "RMC") {
        if (!checksumRmcCorrecto(line)) {
            ++_rmcDiag.checksum_fallidos;
        } else if (!parseRMC(line, r) || !rmcEsRazonable(r)) {
            ++_rmcDiag.rechazadas;
        } else {
            const uint32_t ahoraMs = millis();
            const DateTime utc(r.year, r.month, r.day, r.hour, r.minute, r.second);
            if (_haveLastValidRmc) {
                const DateTime ant(_lastValidRmc.year, _lastValidRmc.month,
                                   _lastValidRmc.day, _lastValidRmc.hour,
                                   _lastValidRmc.minute, _lastValidRmc.second);
                const int32_t delta_s = (int32_t)((int64_t)utc.unixtime() -
                                                  (int64_t)ant.unixtime());
                const uint32_t dt_ms = ahoraMs - _lastValidRmcMs;
                _rmcDiag.intervalo_ms = dt_ms;
                _rmcDiag.ultimo_delta_s = delta_s;
                // Comparacion diagnostica. No se asume exactamente 1 Hz en caso
                // de pausas largas o reacquisicion; tampoco se descarta la RMC.
                if (dt_ms >= 500UL && dt_ms <= 15000UL) {
                    const int32_t esperado = (int32_t)((dt_ms + 500UL) / 1000UL);
                    _rmcDiag.ultimo_error_continuidad_s = delta_s - esperado;
                    if (_rmcDiag.ultimo_error_continuidad_s != 0) {
                        ++_rmcDiag.saltos;
                    }
                } else {
                    _rmcDiag.ultimo_error_continuidad_s = 0;
                }
            }
            ++_rmcDiag.validas;
            ++_rmcDiag.revision;
            _rmcDiag.procesado_us = micros();
            _lastValidRmc = r;
            _haveLastValidRmc = true;
            _lastValidRmcMs = ahoraMs;

        // Base GNSS@PPS: guardo DateTime local y snapshot consistente del contador PPS
        DateTime loc(2000,1,1,0,0,0);
        if (rmcToLocalDateTimeUTCminus3(r, loc)) {
            _lastRmcLocal = loc;
            _haveLastRmcLocal = true;

            noInterrupts();
            unsigned long snap = _ppsPulses;
            interrupts();
            _ppsCountAtLastRmc = snap;
        }
        } // RMC con checksum y fecha validos
    }

    // ---- Parse GGA: num sats + HDOP ----
    if (type == "GGA") {
        _ggaNumSats = toIntSafe(nmeaField(line, 7));
        _ggaHdop    = toFloatSafe(nmeaField(line, 8));
    }

    // ---- Parse GSA: fix type + PDOP/HDOP/VDOP ----
    if (type == "GSA") {
        _gsaFixType = toIntSafe(nmeaField(line, 2));
        _gsaPdop    = toFloatSafe(nmeaField(line, 15));
        _gsaHdop    = toFloatSafe(nmeaField(line, 16));
        _gsaVdop    = toFloatSafe(nmeaField(line, 17));
    }
}

bool GPS_Module::parseRMC(const String& s, RmcData& out) {
    // Validación agnóstica: si no tiene al menos 6 caracteres o no es RMC, se ignora
    if (s.length() < 6 || s.substring(3, 6) != "RMC") return false;

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

    // hhmmss.ss -> tomamos hhmmss (segundos enteros)
    if (timeStr.length() >= 6) {
        out.hour   = timeStr.substring(0, 2).toInt();
        out.minute = timeStr.substring(2, 4).toInt();
        out.second = timeStr.substring(4, 6).toInt();
    }

    // ddmmyy
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

bool GPS_Module::hasLastValidRmc() const { return _haveLastValidRmc; }

RmcDiagnostics GPS_Module::getRmcDiagnostics() const {
    return _rmcDiag; // solo accedida desde loop(), no desde ISR ni tarea WiFi.
}

unsigned long GPS_Module::getLastValidRmcAgeMs() const {
    if (!_haveLastValidRmc) return 0;
    return millis() - _lastValidRmcMs;
}

// ===== GNSS @ PPS estable =====
bool GPS_Module::getGpsLocalAtPps(DateTime& outLocal) const {
    // Necesito una base (RMC válida convertida a local) + PPS andando
    if (!_haveLastRmcLocal) return false;

    unsigned long ppsNow = getPpsPulseCount();
    if (ppsNow == 0) return false;

    // Si por alguna razón ppsNow “retrocede” (reset), no extrapolo
    if (ppsNow < _ppsCountAtLastRmc) return false;

    uint32_t t = _lastRmcLocal.unixtime();
    t += (uint32_t)(ppsNow - _ppsCountAtLastRmc);

    outLocal = DateTime(t);
    return true;
}

// ===== Getters contadores =====
unsigned long GPS_Module::getTxtCount() const { return _cntTXT; }
unsigned long GPS_Module::getRmcCount() const { return _cntRMC; }
unsigned long GPS_Module::getGgaCount() const { return _cntGGA; }
unsigned long GPS_Module::getGsaCount() const { return _cntGSA; }
unsigned long GPS_Module::getOtherCount() const { return _cntOTROS; }

// ===== Getters métricas =====
int   GPS_Module::getGgaNumSats() const { return _ggaNumSats; }
float GPS_Module::getGgaHdop() const { return _ggaHdop; }

int   GPS_Module::getGsaFixType() const { return _gsaFixType; }
float GPS_Module::getGsaPdop() const { return _gsaPdop; }
float GPS_Module::getGsaHdop() const { return _gsaHdop; }
float GPS_Module::getGsaVdop() const { return _gsaVdop; }

// ===== Getters PPS =====
unsigned long GPS_Module::getPpsPulseCount() const {
    noInterrupts();
    unsigned long v = _ppsPulses;
    interrupts();
    return v;
}

unsigned long GPS_Module::getPpsPeriodUs() const {
    noInterrupts();
    unsigned long v = _ppsPeriodUs;
    interrupts();
    return v;
}

bool GPS_Module::hasSeenPps() const {
    noInterrupts();
    bool ok = (_ppsPulses > 0);
    interrupts();
    return ok;
}

unsigned long GPS_Module::getLastPpsUs() const {
    noInterrupts();
    unsigned long v = _ppsLastUsValid;
    interrupts();
    return v;
}

// ================== PPS ISR con filtro “limpio” ==================
void IRAM_ATTR GPS_Module::ppsISR() {
    unsigned long nowUs = micros();

    // Primer flanco: inicializar referencia
    if (!_ppsSeen) {
        _ppsSeen = true;
        _ppsLastUs = nowUs;
        _ppsLastUsValid = 0;
        _ppsPeriodUs = 0;
        _ppsPulses = 0;
        return;
    }

    unsigned long dt = nowUs - _ppsLastUs;

    // 1) Glitch / rebote: demasiado rápido
    if (dt < 200000UL) {
        return; // ignorar, NO tocar _ppsLastUs
    }

    // 2) PPS “real” (aceptar y contar)
    if (dt >= 800000UL && dt <= 1200000UL) {
        _ppsPeriodUs = dt;
        _ppsLastUs = nowUs;          // importantísimo
        _ppsLastUsValid = nowUs;
        _ppsPulses++;
        return;
    }

    // 3) dt demasiado grande: resync
    _ppsLastUs = nowUs;
    // NO incremento _ppsPulses
}