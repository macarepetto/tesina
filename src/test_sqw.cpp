#include <Arduino.h>
#include <string.h>
#include "Pcnt_Extended.h"
#include "Tick_Capture.h"


PcntExtended pcnt;
PeriodicTickCapture sqw(pcnt);
TickEdgeSample sqw_last;

#define PIN_32K_RTC 18
#define PIN_SQW_RTC 19

void setup() {
    Serial.begin(115200);
    delay(300);
    pcnt.begin(PIN_32K_RTC);
    sqw.begin(PIN_SQW_RTC, RISING, 800000, 49152, PULLUP);
}

void loop() {
    
    if (sqw.consume(sqw_last)){
        Serial.printf("CONSUME - Ticks count: %llu - Diff: %lu - Seq: %lu - Discarded: %lu\n", 
        (unsigned long long) sqw_last.ticks,
        (unsigned long) sqw_last.diff,
        (unsigned long) sqw_last.seq,
        (unsigned long) sqw_last.discarded);
    }
    else{
        Serial.print("CONSUME - No hay datos disponibles\n");
    }

    Serial.printf("ELAPSED - Ticks count: %d\n", sqw.readElapsedTicks());
    
  delay(100);
}
 
