#ifndef PHYSICAL_EVENT_MODULE_H
#define PHYSICAL_EVENT_MODULE_H

#include <Arduino.h>

class Telemetry_Queue_Module;

namespace Physical_Event {

// Configura el GPIO del pulsador y su interrupcion.
void begin(int pin);

// Procesa las pulsaciones capturadas y las envia
// a la cola de telemetria, fuera de la ISR.
void publishPending(
    const char* device_id,
    Telemetry_Queue_Module& telemetry
);

}

#endif