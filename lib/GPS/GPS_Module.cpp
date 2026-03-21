#include "GPS_Module.h"

// ========= Static PPS =========
volatile unsigned long GPS_Module::_ppsPulses      = 0;
volatile unsigned long GPS_Module::_ppsLastUs      = 0;
volatile unsigned long GPS_Module::_ppsPeriodUs    = 0;
volatile bool          GPS_Module::_ppsSeen        = false;
volatile unsigned long GPS_Module::_ppsLastUsValid = 0;

GPS_Module::GPS_Module(HardwareSerial& serial, int rxPin, int txPin, int ppsPin, long baud)
    : _serial(serial), _rxPin(rxPin), _txPin(txPin), _ppsPin(ppsPin), _baud(baud) {}

void GPS_Module::begin() {
    _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);

    pinMode(_ppsPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_ppsPin), ppsISR, RISING);
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

    // timeout de sentencia incompleta
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

void GPS_Module::processSentence(const String& line) {
    if (line.length() < 6) return;

    // Contadores por prefijo
    if (line.startsWith("$GPTXT") || line.startsWith("$GNTXT")) _cntTXT++;
    else if (line.startsWith("$GPRMC") || line.startsWith("$GNRMC")) _cntRMC++;
    else if (line.startsWith("$GPGGA") || line.startsWith("$GNGGA")) _cntGGA++;
    else if (line.startsWith("$GPGSA") || line.startsWith("$GNGSA")) _cntGSA++;
    else _cntOTROS++;

    // ---- Parse RMC (solo GP/GN) ----
    RmcData r;
    if (parseRMC(line, r) && rmcEsRazonable(r)) {
        _lastValidRmc = r;
        _haveLastValidRmc = true;
        _lastValidRmcMs = millis();

        // Guardar también como DateTime local + snapshot del contador PPS
        DateTime loc(2000,1,1,0,0,0);
        if (rmcToLocalDateTimeUTCminus3(r, loc)) {
            _lastRmcLocal = loc;
            _haveLastRmcLocal = true;
            _ppsCountAtLastRmc = getPpsPulseCount(); // snapshot consistente
        }
    }

    // ---- Parse GGA: num sats + HDOP ----
    if (line.startsWith("$GPGGA") || line.startsWith("$GNGGA")) {
        _ggaNumSats = toIntSafe(nmeaField(line, 7));
        _ggaHdop    = toFloatSafe(nmeaField(line, 8));
    }

    // ---- Parse GSA: fix type + PDOP/HDOP/VDOP ----
    if (line.startsWith("$GPGSA") || line.startsWith("$GNGSA")) {
        _gsaFixType = toIntSafe(nmeaField(line, 2));
        _gsaPdop    = toFloatSafe(nmeaField(line, 15));
        _gsaHdop    = toFloatSafe(nmeaField(line, 16));
        _gsaVdop    = toFloatSafe(nmeaField(line, 17));
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

    // hhmmss.sss -> tomamos hhmmss
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

unsigned long GPS_Module::getLastValidRmcAgeMs() const {
    if (!_haveLastValidRmc) return 0;
    return millis() - _lastValidRmcMs;
}

// ===== GNSS @ PPS estable =====
bool GPS_Module::getGpsLocalAtPps(DateTime& outLocal) const {
    if (!_haveLastRmcLocal) return false;

    unsigned long ppsNow = getPpsPulseCount();
    if (ppsNow < _ppsCountAtLastRmc) return false; // por seguridad

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