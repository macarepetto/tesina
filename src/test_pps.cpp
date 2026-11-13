#include <Arduino.h>
#include <string.h>
#include "Pcnt_Extended.h"
#include "Tick_Capture.h"


PcntExtended pcnt;
PeriodicTickCapture sqw(pcnt);
TickEdgeSample sqw_last;

PeriodicTickCapture pps(pcnt);
TickEdgeSample pps_last;

#define PIN_32K_RTC 18
#define PIN_SQW_RTC 19
#define PIN_PPS_GPS 27

void setup() {
    Serial.begin(115200);
    delay(300);
    pcnt.begin(PIN_32K_RTC);
    sqw.begin(PIN_SQW_RTC, RISING, 800000, PULLUP);
    pps.begin(PIN_PPS_GPS,RISING, 800000, PULLUP);
}

void loop() {
    // if (sqw.consume(sqw_last)){
    //     Serial.printf("SQW -- Ticks count: %llu - Diff: %lu - Seq: %lu - Discarded: %lu\n", 
    //     (unsigned long long) sqw_last.ticks,
    //     (unsigned long) sqw_last.diff,
    //     (unsigned long) sqw_last.seq,
    //     (unsigned long) sqw_last.discarded);
    // }
    // else{
    //     Serial.print("SQW -- No hay datos disponibles\n");
    // }
    if (pps.consume(pps_last)){
        Serial.printf("PPS -- Ticks count: %llu - Diff: %lu - Seq: %lu - Discarded: %lu\n", 
        (unsigned long long) pps_last.ticks,
        (unsigned long) pps_last.diff,
        (unsigned long) pps_last.seq,
        (unsigned long) pps_last.discarded);
    }
    else{
        Serial.print("PPS -- No hay datos disponibles\n");
    }
  delay(1000);
}
 
