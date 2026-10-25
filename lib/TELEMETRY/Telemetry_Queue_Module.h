#ifndef TELEMETRY_QUEUE_MODULE_H
#define TELEMETRY_QUEUE_MODULE_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "WiFi_Module.h"

// Estadísticas acumuladas desde el arranque; NO son una garantía de entrega al servidor.
struct TelemetryQueueStats {
    uint32_t enqueued = 0;
    uint32_t dropped_full = 0;
    uint32_t dropped_oversize = 0;
    uint32_t tcp_accepted = 0;       // Aceptado por el cliente TCP local (no ACK del servidor)
    uint32_t tcp_offline = 0;        // Descartado para WiFi por falta de conexión
    uint32_t tcp_write_failed = 0;   // Escritura TCP incompleta / fallida
};

// Copia el JSON a una cola, sin operaciones de red ni transmisión Serial en el hilo de medición.
// La tarea de salida se ejecuta en el CORE 0. NO llamar enqueue() desde una ISR.
class Telemetry_Queue_Module {
public:
    static constexpr size_t JSON_CAPACITY = 1800;  // Igual al char json[1800] del main actual.
    static constexpr UBaseType_t QUEUE_LENGTH = 16;

    explicit Telemetry_Queue_Module(WiFi_Module& wifi);

    // Crea la cola y arranca la tarea. NO espera la conexión WiFi.
    bool begin();

    // Retorna false si no se inició, la línea no entra o la cola está llena.
    // La copia del JSON es inmediata; el buffer local del main puede reutilizarse.
    bool enqueue(const char* json);

    TelemetryQueueStats getStats() const;
    UBaseType_t pendingMessages() const;

private:
    struct QueueItem {
        char json[JSON_CAPACITY];
    };

    WiFi_Module& _wifi;
    QueueHandle_t _queue = nullptr;
    TaskHandle_t _taskHandle = nullptr;

    mutable portMUX_TYPE _statsMux = portMUX_INITIALIZER_UNLOCKED;
    TelemetryQueueStats _stats;

    static void taskEntry(void* parameter);
    void taskLoop();
};

#endif
