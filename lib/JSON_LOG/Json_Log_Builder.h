#pragma once

#include <stddef.h>
#include <stdint.h>

#include "Tick_Capture.h"
#include "Kalman_Filter.h"

// Un unico formato de registro para PPS, HOLDOVER y EVENTO_FISICO.
// No lee hardware, no ejecuta Kalman y no envia datos al servidor.
enum class JsonLogType : uint8_t {
    PPS,
    HOLDOVER,
    EVENTO_FISICO
};

// Las tres horas corresponden a los mismos ticks de PCNT.
// Son microsegundos en la escala LOCAL UTC-3 que utilizan los CLOCKS actuales.
struct ClockLogSnapshot {
    bool has_ticks = false;
    uint64_t ticks = 0;

    bool gnss_ok = false;
    int64_t gnss_us = 0;
    bool rtc_ok = false;
    int64_t rtc_us = 0;
    bool kalman_ok = false;
    int64_t kalman_us = 0;
};

struct JsonLogRecord {
    const char* id = nullptr;                     // Obligatorio: identifica el prototipo.
    JsonLogType tipo = JsonLogType::PPS;
    const char* detalle = nullptr;                // Opcional: "ESPERANDO_UTC", "REENGANCHE"...
    uint32_t ms = 0;                             // millis() en el momento de registrar.
    int aging = 0;
    bool rtc_aligned = false;
    bool kalman_trained = false;

    ClockLogSnapshot clocks;

    // PPS: siempre copiar la muestra que devolvio pps.consume(edge).
    bool has_pps = false;
    TickEdgeSample pps;
    bool interval_valid = false;                 // Decidido por el main, NO por el builder.

    // Offset ACUMULADO de los periodos PPS, distinto del offset horario RTC-GNSS.
    bool has_offset_ticks = false;
    int64_t offset_ticks = 0;

    bool kalman_prediccion = false;
    bool kalman_actualizado = false;
    bool has_kalman = false;
    Kalman_State kalman;

    uint32_t ms_since_last_pps = 0;
    uint32_t holdover_steps = 0;

    // Datos disponibles en GPS_Module. Un valor negativo equivale a dato no disponible.
    int gps_sats = -1;
    double gps_pdop = -1.0;
    double gps_hdop = -1.0;
    bool has_utc_validation = false;
    bool utc_valid = false;

    // Evento fisico: captura a implementar, SIN depender del viejo ButtonTickSample.
    // El campo clocks.ticks debe contener los ticks capturados en el evento.
    bool event_captured = false;
    uint32_t event_num = 0;
    uint32_t event_us = 0;                      // micros() de la ISR (se reinicia al desbordar).
    uint32_t event_delay_to_log_us = 0;
    uint32_t event_discarded = 0;
};

// Devuelve false si no hay id, el buffer es insuficiente o falla el formato.
// En ese caso, deja out como cadena vacia: NUNCA enviar JSON truncado.
// Compatible con Telemetry_Queue_Module::JSON_CAPACITY (1800 bytes).
bool buildLogJson(const JsonLogRecord& record, char* out, size_t len);
