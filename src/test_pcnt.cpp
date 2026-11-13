#include <Arduino.h>
#include <string.h>
#include "Pcnt_Extended.h"


PcntExtended pcnt;

#define PIN_32K_RTC 18
u_int64_t prev;

void setup() {
    Serial.begin(115200);
    delay(300);
    pcnt.begin(PIN_32K_RTC);
    prev = pcnt.read();
}

void loop() {
    u_int64_t ticks = pcnt.read();
    u_int64_t diff = ticks - prev;
    prev = ticks;
    Serial.printf("Ticks count: %llu - Diff: %llu\n", (unsigned long long) ticks, (unsigned long long) diff );
    delay(1000);
}
 
