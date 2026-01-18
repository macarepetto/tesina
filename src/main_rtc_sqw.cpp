#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>

RTC_DS3231 rtc;

// Pines
const int PIN_SDA_RTC   = 21;
const int PIN_SCL_RTC   = 22;
const int PIN_SQW_RTC   = 34;
const int PIN_BOTON     = 33;

volatile bool boton_pressed = false;
const unsigned long DEBOUNCE_DELAY = 50;  // in milliseconds
volatile unsigned long lastPressTime = 0;

volatile unsigned long sqw_pulsos = 0;

void IRAM_ATTR buttonISR() {
  unsigned long now = millis();
  if (now - lastPressTime > DEBOUNCE_DELAY) {
    boton_pressed = true;
    lastPressTime = now;
  }
}

// ISR de la señal SQW (1 Hz)
void IRAM_ATTR sqwISR() {
  sqw_pulsos++;   // solo esto, nada de Serial ni millis
}

// Ajustar RTC con hora de compilación (por ahora)
void ajustarRTCConHoraCompilacion() {
  DateTime t(__DATE__, __TIME__);
  rtc.adjust(t);

  Serial.print("RTC ajustado a: ");
  Serial.print(t.year());   Serial.print("/");
  Serial.print(t.month());  Serial.print("/");
  Serial.print(t.day());    Serial.print(" ");
  Serial.print(t.hour());   Serial.print(":");
  Serial.print(t.minute()); Serial.print(":");
  Serial.println(t.second());
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("Iniciando ESP32 + DS3231 con SQW...");

  // I2C del RTC
  Wire.begin(PIN_SDA_RTC, PIN_SCL_RTC);

  if (!rtc.begin()) {
    Serial.println("ERROR: No se encuentra el RTC DS3231 :(");
    while (true) {
      delay(1000);
    }
  }

  // Configurar SQW del DS3231 a 1 Hz
  rtc.writeSqwPinMode(DS3231_SquareWave1Hz);

  // Pin de entrada para SQW (GPIO34 es solo entrada, perfecto para esto)
  pinMode(PIN_SQW_RTC, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_SQW_RTC), sqwISR, RISING);

  // Botón
  pinMode(PIN_BOTON, INPUT_PULLUP);  // botón entre pin y GND
  attachInterrupt(digitalPinToInterrupt(PIN_BOTON), buttonISR, FALLING);

  Serial.println("Listo.");
  Serial.println(" - Apretá el botón para ajustar el RTC a la hora de compilación.");
  Serial.println(" - Observá los pulsos SQW contados en el Serial.");
}

void loop() {
  // Si se apretó el botón, ajustar el RTC
  if (boton_pressed) {
    boton_pressed = false;
    Serial.println("Boton presionado: ajustando RTC a hora de compilacion...");
    ajustarRTCConHoraCompilacion();
  }

  // Mostrar hora del RTC y pulsos SQW cada ~1 segundo
  static unsigned long ultimoPrint = 0;
  unsigned long ahora = millis();

  if (ahora - ultimoPrint >= 1000) {
    ultimoPrint = ahora;

    // Copiar contador de forma segura
    noInterrupts();
    unsigned long pulsos = sqw_pulsos;
    interrupts();

    DateTime now = rtc.now();

    Serial.print("Hora RTC: ");
    Serial.print(now.year());   Serial.print("/");
    Serial.print(now.month());  Serial.print("/");
    Serial.print(now.day());    Serial.print(" ");
    Serial.print(now.hour());   Serial.print(":");
    Serial.print(now.minute()); Serial.print(":");
    Serial.print(now.second());
    Serial.print(" | Pulsos SQW contados: ");
    Serial.println(pulsos);
  }
}
