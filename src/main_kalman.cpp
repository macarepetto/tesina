#include <Arduino.h>
#include "RTC32K_Pcnt_Module.h"
#include "WiFi_Module.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// ================== Pines ==================
static const int PIN_PPS = 27;   // PPS GPS
static const int PIN_32K = 18;   // 32K DS3231

// ================== WiFi / servidor ==================
const char* WIFI_SSID      = "PATAN";
const char* WIFI_PASSWORD  = "autoslocos";
const char* SERVER_IP      = "192.168.0.116";  // <-- tu PC del servidor
const uint16_t SERVER_PORT = 8080;
const char* DEVICE_ID      = "esp32-prototipo-1";

// ================== Módulos ==================
RTC32K_Pcnt_Module rtc32k(PIN_32K);
WiFi_Module telemetry(WIFI_SSID, WIFI_PASSWORD, SERVER_IP, SERVER_PORT, DEVICE_ID);

// ================== Cola ==================
struct Sample {
  uint32_t ms;
  uint32_t pps_seq;
  uint32_t pps_batch;
  uint32_t ticks_per_pps;
  uint64_t ticks_raw;
};
static QueueHandle_t q = nullptr;

// ================== ISR PPS ==================
static volatile uint32_t ppsPending = 0;
static volatile uint32_t lastPpsUs = 0;

void ARDUINO_ISR_ATTR ppsISR() {
  const uint32_t nowUs = (uint32_t)micros();
  const uint32_t dt = nowUs - lastPpsUs;
  lastPpsUs = nowUs;

  if (dt == 0) return;
  if (dt < 200000UL) return; // glitch

  if (dt >= 800000UL && dt <= 1200000UL) {
    ppsPending++;
  }
}

// ================== Estado medición ==================
static uint64_t ticks_prev = 0;
static uint32_t pps_seq = 0;

// ================== Task WiFi (Core 0) ==================
static void wifiTask(void* arg) {
  telemetry.begin();

  for (;;) {
    telemetry.update();

    Sample s;
    while (q && xQueueReceive(q, &s, 0) == pdTRUE) {
      char buf[220];
      snprintf(buf, sizeof(buf),
               "{\"id\":\"%s\",\"ms\":%lu,\"pps_seq\":%lu,\"pps_batch\":%lu,"
               "\"ticks\":%lu,\"ticks_raw\":%llu}",
               DEVICE_ID,
               (unsigned long)s.ms,
               (unsigned long)s.pps_seq,
               (unsigned long)s.pps_batch,
               (unsigned long)s.ticks_per_pps,
               (unsigned long long)s.ticks_raw);

      telemetry.sendLine(String(buf));
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[PPS+32K] Iniciando...");

  // 32K: por las dudas activamos pull-up interno (muchos 32K son open-drain)
  pinMode(PIN_32K, INPUT_PULLUP);

  if (!rtc32k.begin(/*filterTicks=*/0)) {
    Serial.println("ERROR: PCNT no inicio");
    while (true) delay(1000);
  }

  // PPS: normalmente push-pull; si tu PPS es open-drain, poné INPUT_PULLUP
  pinMode(PIN_PPS, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_PPS), ppsISR, RISING);

  ticks_prev = rtc32k.read_counter();

  q = xQueueCreate(256, sizeof(Sample));
  if (!q) {
    Serial.println("ERROR: no pude crear la cola");
    while (true) delay(1000);
  }

  xTaskCreatePinnedToCore(wifiTask, "wifiTask", 4096, nullptr, 1, nullptr, 0);

  Serial.println("Listo. Esperando PPS...");
}

void loop() {
  uint32_t n;
  noInterrupts();
  n = ppsPending;
  ppsPending = 0;
  interrupts();

  if (n == 0) return;

  const uint64_t ticks_raw = rtc32k.read_counter();
  const uint64_t delta = ticks_raw - ticks_prev;
  ticks_prev = ticks_raw;

  const uint32_t ticks_per_pps = (uint32_t)(delta / (uint64_t)n);
  pps_seq += n;

  Sample s;
  s.ms = millis();
  s.pps_seq = pps_seq;
  s.pps_batch = n;
  s.ticks_per_pps = ticks_per_pps;
  s.ticks_raw = ticks_raw;

  if (xQueueSend(q, &s, 0) != pdTRUE) {
    Serial.println("WARN: cola llena -> perdi un sample");
  }

  Serial.printf("PPS n=%lu | ticks_per_pps=%lu | ticks_raw=%llu\n",
                (unsigned long)n,
                (unsigned long)ticks_per_pps,
                (unsigned long long)ticks_raw);
}