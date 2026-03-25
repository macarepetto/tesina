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
const int PIN_SQW_RTC = 19;     // SQW a 1Hz en GPIO19 (permite INPUT_PULLUP)
const int PIN_32K_RTC = 32;     // 32K del DS3231 -> GPIO32

// ================== Pines GPS ==================
const int GPS_RX  = 16;
const int GPS_TX  = 17;
const int GPS_PPS = 27;         // PPS GPS -> GPIO27

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

// Variables para el offset fino
static long offset_us = 0;
static bool offset_valid = false;

// “Latch” GNSS@PPS y RTC@PPS
static bool haveGpsPpsLocal = false;
static DateTime gpsPpsLocal(2000, 1, 1, 0, 0, 0);

static bool haveRtcAtPps = false;
static DateTime rtcAtPps(2000, 1, 1, 0, 0, 0);

static bool haveDiffAtPps = false;
static long rtcMinusGpsPps_s = 0;

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

  Serial.println("Iniciando RTC + GPS(NMEA) + PPS(limpio) + PCNT32K + SQW1Hz + WiFi + BOTON...");

  if (!rtc.begin()) {
    Serial.println("ERROR: No se encuentra el RTC DS3231 :(");
    while (true) delay(1000);
  }

  gps.begin();

  // 32K: NO necesita pull-up
  pinMode(PIN_32K_RTC, INPUT);

  // PCNT 32K
  if (!rtc32k.begin(0)) {
    Serial.println("ERROR: PCNT 32K no pudo iniciar");
  } else {
    Serial.printf("PCNT 32K OK en GPIO%d\n", PIN_32K_RTC);
  }

  telemetry.begin();

  pinMode(PIN_BOTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BOTON), buttonISR, FALLING);

  Serial.println("RTC OK.");
  Serial.println("GPS OK.");
  Serial.println("WiFi telemetry iniciada.");
}

void loop() {
  gps.update();
  telemetry.update();

  // ===== BOTON: ajustar RTC con GPS válido =====
  if (boton_pressed) {
    boton_pressed = false;
    Serial.println("Boton presionado -> intento ajustar RTC con GPS...");

    DateTime gpsLocal(2000, 1, 1, 0, 0, 0);

    if (gps.hasValidRmcRecent(30000) && gps.getLastValidLocalDateTime(gpsLocal)) {
      rtc.adjust(gpsLocal);
      Serial.printf("RTC ajustado con GPS local (UTC-3): %04d/%02d/%02d %02d:%02d:%02d\n",
                    gpsLocal.year(), gpsLocal.month(), gpsLocal.day(),
                    gpsLocal.hour(), gpsLocal.minute(), gpsLocal.second());
    } else {
      Serial.println("No hay RMC valida reciente (NO ajusto RTC).");
    }
  }

  // ===== PPS: si llega PPS nuevo, leemos 32K, calculamos offset y “latcheamos” tiempos =====
  unsigned long pps = gps.getPpsPulseCount();
  if (pps > 0 && pps != lastPps) {
    lastPps = pps;

    uint32_t cycles = rtc32k.readAndReset();

    // sanity
    if (cycles >= 30000UL && cycles <= 36000UL) {
      last32kCycles = cycles;
    } else {
      Serial.printf("[PPS] 32K fuera de rango: %lu (IGNORADO)\n", (unsigned long)cycles);
    }

    // --- NUEVO CÁLCULO DE OFFSET (Fase) ---
    unsigned long pps_us = gps.getLastPpsUs();
    unsigned long sqw_us = rtc.getLastSqwUs();

    // Restamos el tiempo del RTC menos el tiempo del GPS.
    long diff = (long)(sqw_us - pps_us);

    if (abs(diff) < 500000L) {
        offset_us = diff;
        offset_valid = true;
    } else {
        offset_valid = false;
    }
    // ---------------------------------------

    // RTC “capturado” al PPS
    rtcAtPps = rtc.now();
    haveRtcAtPps = true;

    // GNSS@PPS: RMC (segundos) + 1  (aprox: PPS marca el siguiente segundo)
    haveGpsPpsLocal = false;
    if (gps.hasLastValidRmc()) {
      DateTime lastGpsLocal(2000, 1, 1, 0, 0, 0);
      if (gps.getLastValidLocalDateTime(lastGpsLocal)) {
        gpsPpsLocal = DateTime(lastGpsLocal.unixtime() + 1);
        haveGpsPpsLocal = true;
      }
    }

    // diff calculada “en el evento” (no depende de WiFi)
    haveDiffAtPps = (haveRtcAtPps && haveGpsPpsLocal);
    if (haveDiffAtPps) {
      rtcMinusGpsPps_s = (long)rtcAtPps.unixtime() - (long)gpsPpsLocal.unixtime();
    }
  }

  static unsigned long lastPrint = 0;
  static unsigned long lastSend  = 0;

  // ===== Log local =====
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    DateTime nowRtc = rtc.now();
    Serial.printf("RTC: %02d:%02d:%02d | Offset: %ld us | 32K: %ld | PPS: %lu | TCP:%s\n",
      nowRtc.hour(), nowRtc.minute(), nowRtc.second(),
      offset_valid ? offset_us : 0,
      (long)((int32_t)last32kCycles - 32768),
      gps.getPpsPulseCount(),
      telemetry.isServerConnected() ? "OK" : "NO"
    );
  }

  // ===== Envío al servidor (JSONL) =====
  if (millis() - lastSend >= 1000) {
    lastSend = millis();

    DateTime nowRtc = rtc.now();

    String msg = "{";
    msg += "\"id\":\"" + String(DEVICE_ID) + "\"";

    appendDateTimeJson(msg, "rtc", nowRtc);

    msg += ",\"rtc32k_cycles\":" + String((unsigned long)last32kCycles);
    msg += ",\"rtc32k_err\":" + String((long)((int32_t)last32kCycles - 32768));

    msg += ",\"offset_valid\":";
    msg += offset_valid ? "true" : "false";
    if (offset_valid) {
      msg += ",\"offset_us\":" + String(offset_us);
    }

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

    msg += ",\"pps_count\":" + String(gps.getPpsPulseCount());
    msg += ",\"pps_period_us\":" + String(gps.getPpsPeriodUs());
    msg += ",\"pps_last_us\":" + String(gps.getLastPpsUs());

    msg += ",\"rmc_valid\":";
    msg += gps.hasLastValidRmc() ? "true" : "false";

    if (gps.hasLastValidRmc()) {
      DateTime gpsLocal(2000, 1, 1, 0, 0, 0);
      if (gps.getLastValidLocalDateTime(gpsLocal)) {
        appendDateTimeJson(msg, "gps_local", gpsLocal);
        msg += ",\"rmc_age_ms\":" + String(gps.getLastValidRmcAgeMs());
      }
    }

    msg += ",\"gps_pps_valid\":";
    msg += haveGpsPpsLocal ? "true" : "false";
    if (haveGpsPpsLocal) {
      appendDateTimeJson(msg, "gps_pps_local", gpsPpsLocal);
    }

    msg += ",\"rtc_minus_gpspps_valid\":";
    msg += haveDiffAtPps ? "true" : "false";
    if (haveDiffAtPps) {
      msg += ",\"rtc_minus_gpspps_s\":" + String(rtcMinusGpsPps_s);
    }

    msg += "}";

    telemetry.sendLine(msg);
  }
}