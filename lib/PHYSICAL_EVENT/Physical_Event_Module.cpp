
#include "Physical_Event_Module.h"
#include "Json_Log_Builder.h"
#include "Telemetry_Queue_Module.h"

namespace Physical_Event {

void begin(int pin) {
    if (!CaptureInstances::button().begin(pin, 120000)) {
        Serial.println("ERROR: no se pudo iniciar captura del pulsador");
        while (true) delay(1000);
    }
}

void publishPending(
    const char* device_id,
    Telemetry_Queue_Module& telemetry
) {
    ButtonTickSample event;
    uint32_t dropped = 0;

    // Procesar como maximo cuatro eventos por llamada.
    for (int i = 0; i < 4; i++) {

        // 1. Recuperar la pulsacion capturada.
        if (!CaptureInstances::button().consume(event, dropped)) {
            return;
        }

        // 2. Preparar los datos de registro.
        const uint32_t now_ms = millis();
        const uint32_t delay_to_log_us =
            micros() - event.event_us;

        char json[768];

        // 3. Pedirle a Json_Log_Builder que arme el JSON.
        const bool json_ok = buildPhysicalEventJson(
            json,
            sizeof(json),
            device_id,
            event,
            dropped,
            now_ms,
            delay_to_log_us
        );

        if (!json_ok) {
            Serial.println(
                "ERROR: JSON de evento fisico truncado"
            );
            continue;
        }

        // 4. Encolarlo para su transmision.
        if (!telemetry.enqueue(json)) {
            Serial.printf(
                "ERROR: no se pudo encolar evento %lu\n",
                (unsigned long)event.number
            );
        }
    }
}

}
