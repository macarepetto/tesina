#ifndef JSON_LOG_BUILDER_H
#define JSON_LOG_BUILDER_H

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>
#include "Kalman_Filter.h"

struct LogEvent {
    const char* id = "";
    int aging = 0;
    uint32_t ms = 0;
    const char* modo = "";
    bool first = false;

    uint32_t pps_seq = 0;
    uint32_t pps_pending = 0;
    bool has_pps_period_us = false;
    uint32_t pps_period_us = 0;
    uint32_t ms_since_last_pps = 0;
    uint32_t holdover_steps = 0;

    bool offset_medido_disponible = false;
    bool kalman_prediccion = false;
    bool kalman_actualizado = false;

    bool has_ticks = false;
    uint32_t ticks = 0;
    bool has_ticks_window = false;
    uint32_t ticks_window = 0;
    bool has_ticks_raw = false;
    uint32_t ticks_raw = 0;
    bool has_seq_delta = false;
    uint32_t seq_delta = 0;
    bool has_elapsed_us = false;
    uint32_t elapsed_us = 0;

    bool interval_valid = false;
    bool diff_outlier = false;
    uint32_t missed_pps = 0;

    bool has_diff = false;
    int64_t diff = 0;

    bool has_offset = false;
    int64_t offset_ticks = 0;
    double offset_us = 0.0;

    const char* kalman_medicion = "none";

    bool has_kalman_z_phi = false;
    double kalman_z_phi = 0.0;

    bool has_kalman = false;
    double kalman_phi = 0.0;
    double kalman_phi_us = 0.0;
    double kalman_f = 0.0;
    double kalman_p00 = 0.0;
    double kalman_p01 = 0.0;
    double kalman_p10 = 0.0;
    double kalman_p11 = 0.0;
    double kalman_y_phi = 0.0;
    double kalman_k_phi = 0.0;
    double kalman_k_f = 0.0;

    bool has_residual = false;
    double residual_us = 0.0;

    // Campos ya armados por buildRtcGnssJsonFields().
    // Deben venir con coma inicial, por ejemplo:
    // ,"gnss_valido":true,"rtc_hora":"...","gnss_hora":"..."
    const char* rtc_gnss_fields = "";
};

// Función genérica: serializa un LogEvent ya cargado.
void buildLogJson(const LogEvent& event, char* out, size_t len);

// Funciones de alto nivel: el main no debería cargar campo por campo.
void buildHoldoverLogJson(
    char* out,
    size_t len,
    const char* device_id,
    int aging,
    uint32_t now_ms,
    uint32_t seq_prev,
    uint32_t ms_since_last_pps,
    uint32_t holdover_steps,
    int64_t offset_ticks,
    double tick_us,
    const Kalman_State& kalman_state,
    const char* rtc_gnss_fields
);

void buildInitialPpsLogJson(
    char* out,
    size_t len,
    const char* device_id,
    int aging,
    uint32_t now_ms,
    uint32_t seq,
    uint32_t pending,
    uint32_t pps_period_us,
    uint32_t ticks_raw,
    const char* rtc_gnss_fields
);

void buildPpsLogJson(
    char* out,
    size_t len,
    const char* device_id,
    int aging,
    uint32_t now_ms,
    uint32_t seq,
    uint32_t pending,
    uint32_t pps_period_us,
    const char* modo,
    uint32_t holdover_steps,
    bool offset_medido_disponible,
    bool kalman_prediccion,
    bool kalman_actualizado,
    uint32_t ticks,
    uint32_t ticks_window,
    uint32_t ticks_raw,
    uint32_t seq_delta,
    uint32_t elapsed_us,
    bool interval_valid,
    bool diff_outlier,
    uint32_t missed_pps,
    int64_t diff,
    int64_t offset_ticks,
    double tick_us,
    const Kalman_State& kalman_state,
    const char* rtc_gnss_fields
);

#endif