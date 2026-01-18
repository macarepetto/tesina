#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>

RTC_DS3231 rtc;

// Pines
const int PIN_SDA_RTC   = 21;
const int PIN_SCL_RTC   = 22;
const int PIN_BOTON     = 33;

volatile bool boton_pressed = false;
const unsigned long DEBOUNCE_DELAY = 50;  // in milliseconds
volatile unsigned long lastPressTime = 0;


void ARDUINO_ISR_ATTR buttonISR() {
  unsigned long now = millis();
  if (now - lastPressTime > DEBOUNCE_DELAY) {
    boton_pressed = true;
  }
  lastPressTime = now;
}

void ajustarRTCConHoraCompilacion() {
  DateTime t(__DATE__, __TIME__);
  rtc.adjust(t);

  Serial.print("RTC ajustado a: ");
  Serial.print(t.year());  Serial.print("/");
  Serial.print(t.month()); Serial.print("/");
  Serial.print(t.day());   Serial.print(" ");
  Serial.print(t.hour());  Serial.print(":");
  Serial.print(t.minute());Serial.print(":");
  Serial.println(t.second());
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("Iniciando ESP32 + DS3231...");

  Wire.begin(PIN_SDA_RTC, PIN_SCL_RTC);

  if (!rtc.begin()) {
    Serial.println("ERROR: No se encuentra el RTC DS3231 :(");
    while (true) {
      delay(1000);
    }
  }


  pinMode(PIN_BOTON, INPUT_PULLUP);
  attachInterrupt(PIN_BOTON, buttonISR, FALLING);
  Serial.println("Listo. Apretá el botón para poner en hora el RTC.");
}

void loop() {

  if (boton_pressed) {
    Serial.println("Boton presionado: ajustando RTC...");
    ajustarRTCConHoraCompilacion();
    boton_pressed = false;
  }

  DateTime now = rtc.now();

  Serial.print("Hora RTC: ");
  Serial.print(now.year());   Serial.print("/");
  Serial.print(now.month());  Serial.print("/");
  Serial.print(now.day());    Serial.print(" ");
  Serial.print(now.hour());   Serial.print(":");
  Serial.print(now.minute()); Serial.print(":");
  Serial.println(now.second());

  delay(1000);
}