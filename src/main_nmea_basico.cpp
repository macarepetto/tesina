#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>

RTC_DS3231 rtc;

// ================== Pines RTC / Botón ==================
const int PIN_SDA_RTC   = 21;
const int PIN_SCL_RTC   = 22;
const int PIN_SQW_RTC   = 34;
const int PIN_BOTON     = 33;

// ================== Pines GPS UART ==================
static const int GPS_RX = 16;   // GPS TXD -> RX2
static const int GPS_TX = 17;   // GPS RXD -> TX2 (opcional)
HardwareSerial GPSSerial(2);

// ================== Botón con debounce ==================
volatile bool boton_pressed = false;
const unsigned long DEBOUNCE_DELAY = 50;
volatile unsigned long lastPressTime = 0;

void IRAM_ATTR buttonISR() {
  unsigned long now = millis();
  if (now - lastPressTime > DEBOUNCE_DELAY) {
    boton_pressed = true;
    lastPressTime = now;
  }
}

// ================== SQW contador ==================
volatile unsigned long sqw_pulsos = 0;
void IRAM_ATTR sqwISR() { sqw_pulsos++; }

// ================== RTC compile fallback ==================
void ajustarRTCConHoraCompilacion() {
  DateTime t(__DATE__, __TIME__);
  rtc.adjust(t);

  Serial.printf("RTC ajustado (compile): %04d/%02d/%02d %02d:%02d:%02d\n",
                t.year(), t.month(), t.day(),
                t.hour(), t.minute(), t.second());
}

// ================== RMC ==================
struct RmcData {
  bool valid = false;     // status A/V
  int hour = 0, minute = 0, second = 0;
  int day = 0, month = 0, year = 0; // full year (e.g. 2026)
};

// Última RMC válida recibida
RmcData lastValidRmc;
bool haveLastValidRmc = false;
unsigned long lastValidRmcMs = 0;

// Contadores debug (opcionales)
unsigned long cntTXT=0, cntRMC=0, cntGGA=0, cntOTROS=0;

bool parseRMC(const String& s, RmcData& out) {
  if (!(s.startsWith("$GPRMC") || s.startsWith("$GNRMC"))) return false;

  int commas[32];
  int n = 0;
  for (int i = 0; i < (int)s.length() && n < 32; i++) {
    if (s[i] == ',') commas[n++] = i;
  }
  if (n < 9) return true; // es RMC pero incompleta

  auto field = [&](int idx)->String {
    int start = (idx == 0) ? 0 : commas[idx-1] + 1;
    int end   = (idx < n) ? commas[idx] : s.length();
    if (start >= (int)s.length() || end > (int)s.length() || start > end) return "";
    return s.substring(start, end);
  };

  String timeStr = field(1);
  String status  = field(2);
  String dateStr = field(9);

  out.valid = (status.length() > 0 && status[0] == 'A');

  // Hora hhmmss.sss (ignoramos .sss por ahora)
  if (timeStr.length() >= 6) {
    out.hour   = timeStr.substring(0,2).toInt();
    out.minute = timeStr.substring(2,4).toInt();
    out.second = timeStr.substring(4,6).toInt();
  }

  // Fecha ddmmyy
  if (dateStr.length() >= 6) {
    out.day   = dateStr.substring(0,2).toInt();
    out.month = dateStr.substring(2,4).toInt();
    int yy    = dateStr.substring(4,6).toInt();
    out.year  = 2000 + yy; // suficiente para tu caso
  }

  return true;
}

// Check "fuerte" para decir "esto sirve para grabar reloj"
bool rmcEsRazonable(const RmcData& r) {
  if (!r.valid) return false; // status debe ser A

  if (r.year < 2020 || r.year > 2100) return false;
  if (r.month < 1 || r.month > 12) return false;
  if (r.day < 1 || r.day > 31) return false;

  if (r.hour < 0 || r.hour > 23) return false;
  if (r.minute < 0 || r.minute > 59) return false;
  if (r.second < 0 || r.second > 59) return false;

  return true;
}

// Convierte RMC UTC a DateTime local (UTC-3)
bool rmcToLocalDateTimeUTCminus3(const RmcData& r, DateTime& outLocal) {
  if (!rmcEsRazonable(r)) return false;

  DateTime utc(r.year, r.month, r.day, r.hour, r.minute, r.second);

  // Restar 3 horas usando epoch para manejar cambio de día/mes/año
  uint32_t t = utc.unixtime();
  if (t < 3UL * 3600UL) return false; // por seguridad extrema
  t -= 3UL * 3600UL;

  outLocal = DateTime(t);
  return true;
}

// Graba RTC con última RMC válida (local UTC-3)
bool ajustarRTCConUltimaRMCValidaLocal() {
  if (!haveLastValidRmc) return false;

  // opcional: exigir que no sea "vieja" (ej. <= 30 s)
  if (millis() - lastValidRmcMs > 30000UL) {
    Serial.println("Hay RMC valida guardada, pero esta vieja (>30s).");
    return false;
  }

  DateTime localDt(2000,1,1,0,0,0);
  if (!rmcToLocalDateTimeUTCminus3(lastValidRmc, localDt)) return false;

  rtc.adjust(localDt);

  Serial.printf(">> RTC ajustado con GPS VALIDO (hora local UTC-3): %04d/%02d/%02d %02d:%02d:%02d\n",
                localDt.year(), localDt.month(), localDt.day(),
                localDt.hour(), localDt.minute(), localDt.second());
  return true;
}

// ================== Lector NMEA (cierre por '$' + CR/LF) ==================
String nmeaLine;
bool inSentence = false;
unsigned long lastByteMs = 0;

void procesarSentencia(const String& line) {
  if (line.length() < 6) return;

  // Clasificación simple
  if (line.startsWith("$GPTXT") || line.startsWith("$GNTXT")) cntTXT++;
  else if (line.startsWith("$GPRMC") || line.startsWith("$GNRMC")) cntRMC++;
  else if (line.startsWith("$GPGGA") || line.startsWith("$GNGGA")) cntGGA++;
  else cntOTROS++;

  // Parsear RMC si aparece
  RmcData r;
  if (parseRMC(line, r)) {
    Serial.printf("[RMC] %c %02d:%02d:%02d  %02d/%02d/%04d\n",
                  r.valid ? 'A' : 'V',
                  r.hour, r.minute, r.second,
                  r.day, r.month, r.year);

    if (rmcEsRazonable(r)) {
      lastValidRmc = r;
      haveLastValidRmc = true;
      lastValidRmcMs = millis();

      DateTime localTmp(2000,1,1,0,0,0);
      if (rmcToLocalDateTimeUTCminus3(r, localTmp)) {
        Serial.printf("    RMC valida guardada (LOCAL UTC-3): %04d/%02d/%02d %02d:%02d:%02d\n",
                      localTmp.year(), localTmp.month(), localTmp.day(),
                      localTmp.hour(), localTmp.minute(), localTmp.second());
      }
    }
  }

  // Si querés ver TODAS las sentencias, descomentá:
  // Serial.println(line);
}

void cerrarSiHaySentencia() {
  if (inSentence && nmeaLine.length() > 6) {
    procesarSentencia(nmeaLine);
  }
  inSentence = false;
  nmeaLine = "";
}

void leerGPS() {
  while (GPSSerial.available()) {
    char c = (char)GPSSerial.read();
    lastByteMs = millis();

    if (c == '$') {
      cerrarSiHaySentencia();
      inSentence = true;
      nmeaLine = "$";
      continue;
    }

    if (!inSentence) continue;

    if (c == '\r' || c == '\n') {
      cerrarSiHaySentencia();
      continue;
    }

    if (nmeaLine.length() < 200) nmeaLine += c;
    else {
      inSentence = false;
      nmeaLine = "";
    }
  }

  // timeout de sentencia
  if (inSentence && (millis() - lastByteMs) > 200) {
    cerrarSiHaySentencia();
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\nIniciando ESP32 + DS3231 + GPS NMEA (boton=>guardar GPS valido UTC-3)...");

  // RTC
  Wire.begin(PIN_SDA_RTC, PIN_SCL_RTC);
  if (!rtc.begin()) {
    Serial.println("ERROR: No se encuentra el RTC DS3231 :(");
    while (true) delay(1000);
  }

  rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
  pinMode(PIN_SQW_RTC, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_SQW_RTC), sqwISR, RISING);

  // Botón
  pinMode(PIN_BOTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BOTON), buttonISR, FALLING);

  // GPS UART
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS UART2 listo @9600 (RX=16, TX=17)");

  Serial.println("Listo.");
  Serial.println(" - Boton: si hay RMC valida reciente -> graba RTC con hora LOCAL (UTC-3).");
  Serial.println(" - Si no hay RMC valida -> fallback a hora de compilacion.");
}

void loop() {
  // Leer GPS
  leerGPS();

  // Botón: guardar hora
  if (boton_pressed) {
    boton_pressed = false;
    Serial.println("Boton presionado.");

    if (!ajustarRTCConUltimaRMCValidaLocal()) {
      Serial.println("No hay hora GPS valida reciente. Uso hora de compilacion.");
      ajustarRTCConHoraCompilacion();
    }
  }

  // Estado cada 1s
  static unsigned long ultimoPrint = 0;
  unsigned long ahora = millis();

  if (ahora - ultimoPrint >= 1000) {
    ultimoPrint = ahora;

    noInterrupts();
    unsigned long pulsos = sqw_pulsos;
    interrupts();

    DateTime now = rtc.now();

    Serial.printf("Hora RTC local: %04d/%02d/%02d %02d:%02d:%02d | SQW:%lu | TXT:%lu RMC:%lu GGA:%lu OTR:%lu",
                  now.year(), now.month(), now.day(),
                  now.hour(), now.minute(), now.second(),
                  pulsos, cntTXT, cntRMC, cntGGA, cntOTROS);

    if (haveLastValidRmc) {
      unsigned long edad = millis() - lastValidRmcMs;
      Serial.printf(" | ultima RMC valida hace %lums", edad);
    } else {
      Serial.print(" | sin RMC valida");
    }

    Serial.println();
  }
}