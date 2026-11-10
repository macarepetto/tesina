#include "Physical_Event_Module.h"

#include "RTC32K_Pcnt_Capture_Module.h"
#include "Telemetry_Queue_Module.h"

#include <stdio.h>

namespace Physical_Event {

namespace {

// Antirrebote del pulsador: 120 ms.
static constexpr uint32_t DEBOUNCE_US = 120000;

// Cola interna de eventos.
static constexpr uint8_t QUEUE_SIZE = 16;

struct PhysicalEvent {
    uint32_t event_number = 0;
    bool capture_ok = false;

    RTC32K_Pcnt_Capture::ExternalEventSample capture;
};

static PhysicalEvent events[QUEUE_SIZE];

static volatile uint8_t queue_head = 0;
static volatile uint8_t queue_tail = 0;

static volatile uint32_t event_counter = 0;
static volatile uint32_t dropped_events = 0;
static volatile uint32_t last_accepted_us = 0;

static portMUX_TYPE event_mux =
    portMUX_INITIALIZER_UNLOCKED;


// =====================================================
// INTERRUPCION DEL PULSADOR
// =====================================================

static void IRAM_ATTR onButtonPressed() {

    const uint32_t now_us = micros();

    // Ignorar rebotes mecanicos.
    if (
        last_accepted_us != 0 &&
        (uint32_t)(now_us - last_accepted_us) < DEBOUNCE_US
    ) {
        return;
    }

    last_accepted_us = now_us;

    PhysicalEvent event;

    // Capturar el contador RTC y el instante ESP32
    // DENTRO de la interrupcion.
    event.capture_ok =
        RTC32K_Pcnt_Capture::
            captureExternalEventFromIsr(event.capture);

    portENTER_CRITICAL_ISR(&event_mux);

    event.event_number = ++event_counter;

    const uint8_t next_head =
        (queue_head + 1) % QUEUE_SIZE;

    if (next_head != queue_tail) {

        events[queue_head] = event;
        queue_head = next_head;

    } else {

        dropped_events++;
    }

    portEXIT_CRITICAL_ISR(&event_mux);
}


// =====================================================
// EXTRAER UN EVENTO
// =====================================================

static bool popEvent(
    PhysicalEvent& event,
    uint32_t& dropped
) {
    bool available = false;

    portENTER_CRITICAL(&event_mux);

    if (queue_tail != queue_head) {

        event = events[queue_tail];

        queue_tail =
            (queue_tail + 1) % QUEUE_SIZE;

        available = true;
    }

    dropped = dropped_events;

    portEXIT_CRITICAL(&event_mux);

    return available;
}

} // namespace


// =====================================================
// INICIALIZACION
// =====================================================

void begin(int pin) {

    pinMode(pin, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(pin),
        onButtonPressed,
        FALLING
    );
}


// =====================================================
// PUBLICACION DE EVENTOS
// =====================================================

void publishPending(
    const char* device_id,
    Telemetry_Queue_Module& telemetry
) {
    PhysicalEvent event;
    uint32_t dropped = 0;

    // Procesar como maximo cuatro eventos por llamada.
    for (int i = 0; i < 4; i++) {

        if (!popEvent(event, dropped)) {
            return;
        }

        const auto& c = event.capture;

        const bool has_pps =
            c.last_pps_seq != 0;

        const uint32_t delta_pps_us =
            c.event_micros - c.last_pps_micros;

        const uint32_t delay_to_log_us =
            micros() - c.event_micros;

        char event_ticks[24];
        char delta_pps[24];
        char delta_ticks[24];

        // Contador RTC en el instante del evento.
        if (event.capture_ok) {

            snprintf(
                event_ticks,
                sizeof(event_ticks),
                "%lu",
                (unsigned long)c.ticks_raw
            );

        } else {

            snprintf(
                event_ticks,
                sizeof(event_ticks),
                "null"
            );
        }

        // Tiempo desde el ultimo PPS capturado.
        if (has_pps) {

            snprintf(
                delta_pps,
                sizeof(delta_pps),
                "%lu",
                (unsigned long)delta_pps_us
            );

        } else {

            snprintf(
                delta_pps,
                sizeof(delta_pps),
                "null"
            );
        }

        // Pulsos RTC transcurridos desde el ultimo PPS.
        if (has_pps && event.capture_ok) {

            const int32_t difference =
                (int32_t)(
                    c.ticks_raw -
                    c.last_pps_ticks_raw
                );

            snprintf(
                delta_ticks,
                sizeof(delta_ticks),
                "%ld",
                (long)difference
            );

        } else {

            snprintf(
                delta_ticks,
                sizeof(delta_ticks),
                "null"
            );
        }

        char json[768];

        const int written = snprintf(
            json,
            sizeof(json),

            "{"
            "\"id\":\"%s\","
            "\"modo\":\"EVENTO_FISICO\","
            "\"ms\":%lu,"
            "\"evento_num\":%lu,"
            "\"evento_esp_us\":%lu,"
            "\"evento_ticks_raw\":%s,"
            "\"evento_pps_seq\":%lu,"
            "\"evento_pps_micros\":%lu,"
            "\"evento_pps_ticks_raw\":%lu,"
            "\"evento_delta_pps_us\":%s,"
            "\"evento_delta_ticks\":%s,"
            "\"evento_captura_ok\":%s,"
            "\"evento_retardo_registro_us\":%lu,"
            "\"evento_descartados\":%lu"
            "}",

            device_id,

            (unsigned long)millis(),

            (unsigned long)event.event_number,

            (unsigned long)c.event_micros,

            event_ticks,

            (unsigned long)c.last_pps_seq,

            (unsigned long)c.last_pps_micros,

            (unsigned long)c.last_pps_ticks_raw,

            delta_pps,

            delta_ticks,

            event.capture_ok ? "true" : "false",

            (unsigned long)delay_to_log_us,

            (unsigned long)dropped
        );

        if (
            written < 0 ||
            (size_t)written >= sizeof(json)
        ) {
            Serial.println(
                "ERROR: JSON de evento fisico truncado"
            );
            continue;
        }

        // El JSON se envia fuera de la ISR.
        if (!telemetry.enqueue(json)) {

            Serial.printf(
                "ERROR: no se pudo encolar evento %lu\n",
                (unsigned long)event.event_number
            );
        }
    }
}

}