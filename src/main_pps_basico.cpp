#include <Arduino.h>

const int PIN_PPS_GPS = 35;

volatile unsigned long pps_pulsos = 0;
volatile unsigned long pps_last_us = 0;
volatile unsigned long pps_period_us = 0;
volatile bool pps_seen = false;

void IRAM_ATTR ppsISR() {
  unsigned long nowUs = micros();
  pps_pulsos++;
  if (pps_seen) pps_period_us = nowUs - pps_last_us;
  else pps_seen = true;
  pps_last_us = nowUs;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_PPS_GPS, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_PPS_GPS), ppsISR, RISING);

  Serial.printf("Test PPS en GPIO%d\n", PIN_PPS_GPS);
}

void loop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    noInterrupts();
    unsigned long n = pps_pulsos;
    unsigned long per = pps_period_us;
    bool seen = pps_seen;
    interrupts();

    Serial.printf("PPS count=%lu", n);
    if (seen) Serial.printf(" | period=%lu us", per);
    else Serial.print(" | sin pulsos aun");
    Serial.println();
  }
}