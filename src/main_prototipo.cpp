// src/main_prototipo.cpp

#include <Arduino.h>
#include <RTClib.h>
#include "RTC_Module.h"
#include "GPS_Module.h"
#include "WiFi_Module.h"
#include "RTC32K_Pcnt_Module.h"

// Librerías de FreeRTOS (Para el Dual Core y las Colas)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

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

// ================== Objetos de FreeRTOS ==================
QueueHandle_t telemetryQueue; // El "buzón" entre los dos núcleos
void tareaWiFi(void *pvParameters); // Declaración de la tarea del Core 0

// ================== Estado derivado ==================
static unsigned long lastPps = 0;
static uint32_t last32kCycles = 0;

// Gatillo para el envío por hardware
static unsigned long lastSqwTrigger = 0;

// Gatillo para sincronización exacta
static bool pending_rtc_sync = false;
static bool flag_sync_event = false;

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

// ================== Funciones Alta Precisión ==================
uint32_t getRtcMicros() {
    unsigned long lastSqw = rtc.getLastSqwUs();
    if (lastSqw == 0) return 0;
    return (uint32_t)(micros() - lastSqw) % 1000000UL; 
}

uint32_t getGpsMicros() {
    unsigned long lastPps = gps.getLastPpsUs();
    if (lastPps == 0) return 0;
    return (uint32_t)(micros() - lastPps) % 1000000UL;
}

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

  Serial.println("Iniciando Prototipo (Dual Core: Core1->Metrologia, Core0->WiFi)...");

  if (!rtc.begin()) {
    Serial.println("ERROR: No se encuentra el RTC DS3231 :(");
    while (true) delay(1000);
  }

  // Agrandamos el buffer del GPS para evitar embotellamientos
  Serial2.setRxBufferSize(1024);
  gps.begin();

  pinMode(PIN_32K_RTC, INPUT);
  if (!rtc32k.begin(0)) {
    Serial.println("ERROR: PCNT 32K no pudo iniciar");
  } else {
    Serial.printf("PCNT 32K OK en GPIO%d\n", PIN_32K_RTC);
  }

  telemetry.begin();
  
  pinMode(PIN_BOTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BOTON), buttonISR, FALLING);

  // ================== CONFIGURACIÓN DUAL CORE ==================
  // 1. Creamos el buzón (Queue) con capacidad para 10 mensajes
  telemetryQueue = xQueueCreate(10, sizeof(String*));

  // 2. Creamos la Tarea WiFi y la anclamos estrictamente al CORE 0
  xTaskCreatePinnedToCore(
    tareaWiFi,        // Función de la tarea
    "TareaWiFi",      // Nombre para debug
    8192,             // Tamaño de la pila (Stack)
    NULL,             // Parámetros
    1,                // Prioridad
    NULL,             // Handle
    0                 // <--- NÚCLEO 0
  );

  Serial.println("Setup OK. Esperando satélites...");
}

// ======================================================================
// === CORE 0: ESTA TAREA SE ENCARGA EXCLUSIVAMENTE DEL WIFI Y TCP ====
// ======================================================================
void tareaWiFi(void *pvParameters) {
  String* msgRecibido;
  
  for(;;) {
    // Mantener la conexión activa (si se cae el WiFi, se frena ACÁ, no afecta al reloj)
    telemetry.update(); 
    
    // Revisar el buzón. Espera hasta 10ms para ver si el loop() mandó un JSON nuevo
    if (xQueueReceive(telemetryQueue, &msgRecibido, 10 / portTICK_PERIOD_MS) == pdPASS) {
      telemetry.sendLine(*msgRecibido); // Enviar por TCP
      delete msgRecibido;               // ¡Muy importante! Liberar la memoria RAM
    }
  }
}

// ======================================================================
// === CORE 1: ESTE LOOP ES EL "MASTER" DE TIEMPO REAL ==================
// ======================================================================
void loop() {
  gps.update();

  // ===== BOTON: Solo arma el gatillo =====
  if (boton_pressed) {
    boton_pressed = false;
    if (gps.hasValidRmcRecent(1200)) { 
        pending_rtc_sync = true;
    }
  }

  // ===== PPS: Si llega PPS nuevo, leemos 32K y latcheamos tiempos =====
  unsigned long pps = gps.getPpsPulseCount();
  if (pps > 0 && pps != lastPps) {
    lastPps = pps;

    haveGpsPpsLocal = false;
    if (gps.hasValidRmcRecent(1200)) { 
      DateTime lastGpsLocal(2000, 1, 1, 0, 0, 0);
      if (gps.getLastValidLocalDateTime(lastGpsLocal)) {
        gpsPpsLocal = DateTime(lastGpsLocal.unixtime() + 1);
        haveGpsPpsLocal = true;
      }
    }

    // --- EJECUCIÓN DEL GATILLO DE SINCRONIZACIÓN FINA ---
    if (pending_rtc_sync && haveGpsPpsLocal) {
        rtc.adjust(gpsPpsLocal);
        pending_rtc_sync = false;
        flag_sync_event = true; 
        return; 
    }

    // --- CÁLCULOS DE ERROR TEMPORAL Y DERIVA ---
    uint32_t cycles = rtc32k.readAndReset();
    if (cycles >= 30000UL && cycles <= 36000UL) {
      last32kCycles = cycles;
    }

    unsigned long pps_us = gps.getLastPpsUs();
    unsigned long sqw_us = rtc.getLastSqwUs();

    long diff = (long)(sqw_us - pps_us);

    if (abs(diff) < 500000L) {
        offset_us = diff;
        offset_valid = true;
    } else {
        offset_valid = false;
    }

    rtcAtPps = rtc.now();
    haveRtcAtPps = true;

    haveDiffAtPps = (haveRtcAtPps && haveGpsPpsLocal);
    if (haveDiffAtPps) {
      rtcMinusGpsPps_s = (long)rtcAtPps.unixtime() - (long)gpsPpsLocal.unixtime();
    }
  }

  // ===== DETECCIÓN DE PÉRDIDA DE SEÑAL GNSS (WATCHDOG) =====
  if (micros() - gps.getLastPpsUs() > 1500000UL) {
      offset_valid = false;  
      haveDiffAtPps = false; 
      pending_rtc_sync = false; 
  }

  // ===== ARMADO DE LOGS GUIADO POR EL SQW (Como pidió el profe) ====
  unsigned long currentSqw = rtc.getSqwPulses();
  
  if (currentSqw > 0 && currentSqw != lastSqwTrigger) {
    lastSqwTrigger = currentSqw;

    DateTime nowRtc = rtc.now();
    uint32_t rtcUs = getRtcMicros();
    uint32_t gpsUs = getGpsMicros();

    String msg = "{";
    msg += "\"id\":\"" + String(DEVICE_ID) + "\"";

    msg += ",\"sync_event\":";
    msg += flag_sync_event ? "true" : "false";
    flag_sync_event = false;

    appendDateTimeJson(msg, "rtc", nowRtc);

    char rtcPreciseBuf[32];
    sprintf(rtcPreciseBuf, "%02d:%02d:%02d.%06lu", nowRtc.hour(), nowRtc.minute(), nowRtc.second(), (unsigned long)rtcUs);
    msg += ",\"rtc_precisa\":\"" + String(rtcPreciseBuf) + "\"";

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
    msg += gps.hasValidRmcRecent(2000) ? "true" : "false";

    if (gps.hasValidRmcRecent(2000)) {
      DateTime gpsLocal(2000, 1, 1, 0, 0, 0);
      if (gps.getLastValidLocalDateTime(gpsLocal)) {
        appendDateTimeJson(msg, "gps_local", gpsLocal);
        
        char gpsPreciseBuf[32];
        sprintf(gpsPreciseBuf, "%02d:%02d:%02d.%06lu", gpsLocal.hour(), gpsLocal.minute(), gpsLocal.second(), (unsigned long)gpsUs);
        msg += ",\"gps_precisa\":\"" + String(gpsPreciseBuf) + "\"";
        
        msg += ",\"rmc_age_ms\":" + String(gps.getLastValidRmcAgeMs());
      }
    }

    msg += ",\"gps_pps_valid\":";
    msg += haveGpsPpsLocal ? "true" : "false";
    if (haveGpsPpsLocal) {
      appendDateTimeJson(msg, "gps_pps_local", gpsPpsLocal);
      
      int64_t rtc_total_us = (int64_t)nowRtc.unixtime() * 1000000LL + rtcUs;
      int64_t gps_total_us = (int64_t)gpsPpsLocal.unixtime() * 1000000LL + gpsUs;
      int64_t error_absoluto_us = rtc_total_us - gps_total_us;

      char errBuf[128];
      snprintf(errBuf, sizeof(errBuf), ",\"error_total_us\":%lld,\"error_total_s\":%.6f", 
               error_absoluto_us, 
               (double)error_absoluto_us / 1000000.0);
      msg += String(errBuf);
    }

    msg += "}\n";

    // ================================================================
    // >>> ENVÍO DE DATOS AL CORE 0 (Metiendo el mensaje en el buzón)
    // ================================================================
    String* msgPtr = new String(msg); // Creamos un paquete dinámico
    
    // Si el buzón no está lleno, mandamos el puntero a la cola
    if (xQueueSend(telemetryQueue, &msgPtr, 0) != pdPASS) {
        // Si el buzón se llenó (porque el WiFi está súper caído), descartamos el paquete actual
        delete msgPtr; 
    }
  }
}