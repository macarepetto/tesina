#include <Arduino.h>

// Global variables for the button
const uint8_t buttonPin = 33;
volatile int32_t counter = 0;
volatile bool pressed = false;

const unsigned long DEBOUNCE_DELAY = 50;  // in milliseconds
volatile unsigned long lastPressTime = 0;

// Interrupt Service Routine (ISR)
void ARDUINO_ISR_ATTR buttonISR() {
  unsigned long now = millis();
  if (now - lastPressTime > DEBOUNCE_DELAY) {
    counter++;
    pressed = true;
  }
  lastPressTime = now;
}


void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(buttonPin, buttonISR, RISING);
  Serial.println("Press the button on GPIO 33.");
}

void loop() {
  if (pressed) {
    Serial.print("Button pressed ");
    Serial.print(counter);
    Serial.println(" times.");
    pressed = false;
  }
  delay(10);
}