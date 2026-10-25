#include "Telemetry_Queue_Module.h"
#include <string.h>

Telemetry_Queue_Module::Telemetry_Queue_Module(WiFi_Module& wifi)
    : _wifi(wifi) {}

bool Telemetry_Queue_Module::begin() {
    if (_queue != nullptr) {
        return true;  // Ya iniciada.
    }

    _queue = xQueueCreate(QUEUE_LENGTH, sizeof(QueueItem));
    if (_queue == nullptr) {
        return false;
    }

    // ESP32 clásico dual-core. Prioridad baja: no compite con las mediciones del CORE 1.
    const BaseType_t result = xTaskCreatePinnedToCore(
        taskEntry,
        "Telemetria",
        8192,      // ESP32/ESP-IDF: tamaño de pila expresado en bytes.
        this,
        1,
        &_taskHandle,
        0          // CORE 0
    );

    if (result != pdPASS) {
        vQueueDelete(_queue);
        _queue = nullptr;
        _taskHandle = nullptr;
        return false;
    }
    return true;
}

bool Telemetry_Queue_Module::enqueue(const char* json) {
    if (_queue == nullptr || json == nullptr) {
        return false;
    }

    // El JSON que construye snprintf() en el main debe terminar en '\0'.
    // Nunca truncamos silenciosamente el JSON: un objeto truncado sería inválido.
    const size_t length = strnlen(json, JSON_CAPACITY);
    if (length >= JSON_CAPACITY) {
        portENTER_CRITICAL(&_statsMux);
        ++_stats.dropped_oversize;
        portEXIT_CRITICAL(&_statsMux);
        return false;
    }

    QueueItem item = {};
    memcpy(item.json, json, length + 1);

    // 0 ticks: NUNCA esperar a que haya espacio. Si está llena, descartar el nuevo.
    const bool ok = xQueueSend(_queue, &item, 0) == pdTRUE;
    portENTER_CRITICAL(&_statsMux);
    if (ok) {
        ++_stats.enqueued;
    } else {
        ++_stats.dropped_full;
    }
    portEXIT_CRITICAL(&_statsMux);
    return ok;
}

TelemetryQueueStats Telemetry_Queue_Module::getStats() const {
    portENTER_CRITICAL(&_statsMux);
    const TelemetryQueueStats copy = _stats;
    portEXIT_CRITICAL(&_statsMux);
    return copy;
}

UBaseType_t Telemetry_Queue_Module::pendingMessages() const {
    return _queue ? uxQueueMessagesWaiting(_queue) : 0;
}

void Telemetry_Queue_Module::taskEntry(void* parameter) {
    static_cast<Telemetry_Queue_Module*>(parameter)->taskLoop();
}

void Telemetry_Queue_Module::taskLoop() {
    // Toda la comunicación WiFi/TCP se hace en CORE 0.
    // La conexión inicial puede bloquear, pero no detiene el loop de medición.
    _wifi.begin();

    QueueItem item = {};
    for (;;) {
        _wifi.update();

        if (xQueueReceive(_queue, &item, pdMS_TO_TICKS(50)) != pdTRUE) {
            continue;
        }

        // El mensaje se procesa una sola vez. Si no hay conexión,
        // se cuenta el descarte; no se bloquea el loop para reintentar.
        if (!_wifi.isWifiConnected() || !_wifi.isServerConnected()) {
            portENTER_CRITICAL(&_statsMux);
            ++_stats.tcp_offline;
            portEXIT_CRITICAL(&_statsMux);
        } else if (_wifi.sendLine(item.json)) {
            portENTER_CRITICAL(&_statsMux);
            ++_stats.tcp_accepted;
            portEXIT_CRITICAL(&_statsMux);
        } else {
            portENTER_CRITICAL(&_statsMux);
            ++_stats.tcp_write_failed;
            portEXIT_CRITICAL(&_statsMux);
        }
    }
}
