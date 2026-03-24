// src/main_prototipo.cpp

#include <Arduino.h>
#include <RTClib.h>
#include "RTC_Module.h"
#include "GPS_Module.h"
#include "WiFi_Module.h"
#include "RTC32K_Pcnt_Module.h"

// ================== Pines RTC ==================
const int PIN_SDA_RTC = 21;
const int PIN_SCL_RTC = 22;
const int PIN_SQW_RTC = -1;     // SQW deshabilitado (opcional)
const int PIN_32K_RTC = 32;     // salida 32K del DS3231 -> GPIO32

// ================== Pines GPS ==================
const int GPS_RX  = 16;
const int GPS_TX  = 17;
const int GPS_PPS = 27;         // PPS del GPS -> GPIO27
// ================== Botón ==================
const int PIN_BOTON = 33;

volatile bool boton_pressed = false;
volatile unsigned long lastPressTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

void IRAM_ATTR buttonISR() {
  unsigned long now = millis();
  if (now - lastPressTime > DEBOUNCE_DELAY) {
    boton_pressed = true;
    lastPressTime = now;
  }
}

// ================== WiFi / servidor ==================
const char* WIFI_SSID     = "PATAN";
const char* WIFI_PASSWORD = "autoslocos";
const char* SERVER_IP     = "192.168.0.119";
//const char* WIFI_SSID = "W_IF012"; 
//const char* WIFI_PASSWORD = "if0122022"; 
//const char* SERVER_IP = "192.168.40.68"; 
//const char* WIFI_SSID = "motorola one macro 6295"; 
//const char* WIFI_PASSWORD = "21f112cb2225";
//const char* SERVER_IP = "10.124.105.91";
const uint16_t SERVER_PORT = 8080;
const char* DEVICE_ID = "esp32-prototipo-1";

// ================== Objetos ==================
RTC_Module rtc(PIN_SDA_RTC, PIN_SCL_RTC, PIN_SQW_RTC);
GPS_Module gps(Serial2, GPS_RX, GPS_TX, GPS_PPS, 9600);
WiFi_Module telemetry(WIFI_SSID, WIFI_PASSWORD, SERVER_IP, SERVER_PORT, DEVICE_ID);
RTC32K_Pcnt_Module rtc32k(PIN_32K_RTC);

// ================== Estado derivado ==================
static unsigned long lastPps = 0;
static uint32_t last32kCycles = 0;

// “Latch” GNSS@PPS (hora asignada al último PPS válido)
static bool haveGpsPpsLocal = false;
static DateTime gpsPpsLocal(2000, 1, 1, 0, 0, 0);

static void appendDateTimeJson(String& msg, const char* key, const DateTime& dt) {
  msg += ",\"";
  msg += key;
  msg += "\":\"";
  msg += String(dt.year()) + "-";
  if (dt.month() < 10) msg += "0";
  msg += String(dt.month()) + "-";
  if (dt.day() < 10) msg += "0";
  msg += String(dt.day()) + " ";
  if (dt.hour() < 10) msg += "0";
  msg += String(dt.hour()) + ":";
  if (dt.minute() < 10) msg += "0";
  msg += String(dt.minute()) + ":";
  if (dt.second() < 10) msg += "0";
  msg += String(dt.second());
  msg += "\"";
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("Iniciando RTC + GPS(NMEA) + PPS(limpio) + PCNT32K + WiFi + BOTON...");

  // RTC
  if (!rtc.begin()) {
    Serial.println("ERROR: No se encuentra el RTC DS3231 :(");
    while (true) delay(1000);
  }

  // GPS + PPS
  gps.begin();

  pinMode(PIN_32K_RTC, INPUT_PULLUP);

  // PCNT 32K
  // Recomendación: arrancá con 0 (filtro apagado) y si hay glitches probá 100/200/400
  if (!rtc32k.begin(0)) {
    Serial.println("ERROR: PCNT 32K no pudo iniciar");
  } else {
    Serial.println("PCNT 32K OK en GPIO34");
  }

  // WiFi + TCP
  telemetry.begin();

  // Botón
  pinMode(PIN_BOTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BOTON), buttonISR, FALLING);

  Serial.println("RTC OK (NO se ajusta automaticamente).");
  Serial.println("GPS OK.");
  Serial.println("WiFi telemetry iniciada.");
  Serial.println("Boton: ajusta RTC con GPS si hay RMC valida; sino compile time.");
}

  void loop() {
    gps.update();
    telemetry.update();

    // ===== BOTON: ajustar RTC con GPS válido =====
    if (boton_pressed) {
      boton_pressed = false;
      Serial.println("Boton presionado -> intento ajustar RTC con GPS...");

      DateTime gpsLocal(2000, 1, 1, 0, 0, 0);

      // Exigir RMC válida reciente (<= 30s)
      if (gps.hasValidRmcRecent(30000) && gps.getLastValidLocalDateTime(gpsLocal)) {
        rtc.adjust(gpsLocal);
        Serial.printf("RTC ajustado con GPS local (UTC-3): %04d/%02d/%02d %02d:%02d:%02d\n",
                      gpsLocal.year(), gpsLocal.month(), gpsLocal.day(),
                      gpsLocal.hour(), gpsLocal.minute(), gpsLocal.second());
      } else {
        Serial.println("No hay RMC valida reciente.");
      }
    }

  // ===== PPS: si llega un PPS válido nuevo, leemos 32K y latch GNSS@PPS =====
  unsigned long pps = gps.getPpsPulseCount();
  if (pps > 0 && pps != lastPps) {
    lastPps = pps;

    uint32_t cycles = rtc32k.readAndReset();

    // SANITY CHECK: descartar mediciones locas (por glitch/ruido/ISR)
    if (cycles >= 30000UL && cycles <= 36000UL) {
      last32kCycles = cycles;
    } else {
      // no pisamos last32kCycles; solo avisamos por Serial
      Serial.printf("[PPS] 32K fuera de rango: %lu (IGNORADO)\n", (unsigned long)cycles);
    }

    int32_t err = (int32_t)last32kCycles - 32768;
    Serial.printf("[PPS] 32K cycles/1s(GNSS): %lu (err=%ld)\n",
                  (unsigned long)last32kCycles, (long)err);

    // Latch GNSS@PPS:
    // tomamos la última RMC válida (segundos) y asumimos PPS ~ inicio del próximo segundo.
    haveGpsPpsLocal = false;
    if (gps.hasLastValidRmc()) {
      DateTime lastGpsLocal(2000, 1, 1, 0, 0, 0);
      if (gps.getLastValidLocalDateTime(lastGpsLocal)) {
        gpsPpsLocal = DateTime(lastGpsLocal.unixtime() + 1);
        haveGpsPpsLocal = true;
      }
    }
  }

  static unsigned long lastPrint = 0;
  static unsigned long lastSend  = 0;

  // ===== Log local =====
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    DateTime nowRtc = rtc.now();

    Serial.printf("Hora RTC: %04d/%02d/%02d %02d:%02d:%02d | 32K:%lu err:%ld | TXT:%lu RMC:%lu GGA:%lu OTR:%lu",
                  nowRtc.year(), nowRtc.month(), nowRtc.day(),
                  nowRtc.hour(), nowRtc.minute(), nowRtc.second(),
                  (unsigned long)last32kCycles,
                  (long)((int32_t)last32kCycles - 32768),
                  gps.getTxtCount(),
                  gps.getRmcCount(),
                  gps.getGgaCount(),
                  gps.getOtherCount());

    if (gps.getPpsPulseCount() > 0) {
      Serial.printf(" | PPS_OK:%lu (T=%lu us) last_us=%lu",
                    gps.getPpsPulseCount(),
                    gps.getPpsPeriodUs(),
                    gps.getLastPpsUs());
    } else {
      Serial.print(" | PPS_OK:0");
    }

    if (gps.hasLastValidRmc()) {
      DateTime gpsLocal(2000, 1, 1, 0, 0, 0);
      if (gps.getLastValidLocalDateTime(gpsLocal)) {
        Serial.printf(" | GPS(local RMC): %04d/%02d/%02d %02d:%02d:%02d age:%lums",
                      gpsLocal.year(), gpsLocal.month(), gpsLocal.day(),
                      gpsLocal.hour(), gpsLocal.minute(), gpsLocal.second(),
                      gps.getLastValidRmcAgeMs());
      }
    } else {
      Serial.print(" | sin RMC valida");
    }

    if (haveGpsPpsLocal) {
      Serial.printf(" | GPS@PPS: %04d/%02d/%02d %02d:%02d:%02d",
                    gpsPpsLocal.year(), gpsPpsLocal.month(), gpsPpsLocal.day(),
                    gpsPpsLocal.hour(), gpsPpsLocal.minute(), gpsPpsLocal.second());
    }

    Serial.printf(" | WiFi:%s | TCP:%s",
                  telemetry.isWifiConnected() ? "OK" : "NO",
                  telemetry.isServerConnected() ? "OK" : "NO");

    Serial.println();
  }

  // ===== Envío al servidor (JSONL) =====
  if (millis() - lastSend >= 1000) {
    lastSend = millis();

    DateTime nowRtc = rtc.now();

    String msg = "{";
    msg += "\"id\":\"" + String(DEVICE_ID) + "\"";

    // RTC
    msg += ",\"rtc\":\"";
    msg += String(nowRtc.year()) + "-";
    if (nowRtc.month() < 10) msg += "0";
    msg += String(nowRtc.month()) + "-";
    if (nowRtc.day() < 10) msg += "0";
    msg += String(nowRtc.day()) + " ";
    if (nowRtc.hour() < 10) msg += "0";
    msg += String(nowRtc.hour()) + ":";
    if (nowRtc.minute() < 10) msg += "0";
    msg += String(nowRtc.minute()) + ":";
    if (nowRtc.second() < 10) msg += "0";
    msg += String(nowRtc.second());
    msg += "\"";

    // 32K por PPS
    msg += ",\"rtc32k_cycles\":" + String((unsigned long)last32kCycles);
    msg += ",\"rtc32k_err\":" + String((long)((int32_t)last32kCycles - 32768));

    // contadores NMEA + métricas
    msg += ",\"txt\":" + String(gps.getTxtCount());
    msg += ",\"rmc\":" + String(gps.getRmcCount());
    msg += ",\"gga\":" + String(gps.getGgaCount());
    msg += ",\"gga_sats\":" + String(gps.getGgaNumSats());
    msg += ",\"gga_hdop\":" + String(gps.getGgaHdop(), 2);
    msg += ",\"gsa_fix\":" + String(gps.getGsaFixType());
    msg += ",\"pdop\":" + String(gps.getGsaPdop(), 2);
    msg += ",\"hdop\":" + String(gps.getGsaHdop(), 2);
    msg += ",\"vdop\":" + String(gps.getGsaVdop(), 2);
    msg += ",\"otr\":" + String(gps.getOtherCount());

    // PPS
    msg += ",\"pps_count\":" + String(gps.getPpsPulseCount());
    msg += ",\"pps_period_us\":" + String(gps.getPpsPeriodUs());
    msg += ",\"pps_last_us\":" + String(gps.getLastPpsUs());

    // GPS local desde RMC
    msg += ",\"rmc_valid\":";
    msg += gps.hasLastValidRmc() ? "true" : "false";

    if (gps.hasLastValidRmc()) {
      DateTime gpsLocal(2000, 1, 1, 0, 0, 0);
      if (gps.getLastValidLocalDateTime(gpsLocal)) {
        appendDateTimeJson(msg, "gps_local", gpsLocal);
        msg += ",\"rmc_age_ms\":" + String(gps.getLastValidRmcAgeMs());
      }
    }

    // GNSS @ PPS (latch)
    msg += ",\"gps_pps_valid\":";
    msg += haveGpsPpsLocal ? "true" : "false";

    if (haveGpsPpsLocal) {
      appendDateTimeJson(msg, "gps_pps_local", gpsPpsLocal);

      long diff_s = (long)nowRtc.unixtime() - (long)gpsPpsLocal.unixtime();
      msg += ",\"rtc_minus_gpspps_s\":" + String(diff_s);
    }

    msg += "}";

    telemetry.sendLine(msg);
  }
}