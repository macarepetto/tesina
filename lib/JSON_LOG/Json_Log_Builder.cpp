#include "Json_Log_Builder.h"

#include <stdio.h>

static const char* jsonBool(bool value) {
    return value ? "true" : "false";
}

static void jsonUInt32(char* out, size_t len, bool valid, uint32_t value) {
    if (!valid) {
        snprintf(out, len, "null");
        return;
    }
    snprintf(out, len, "%lu", (unsigned long)value);
}

static void jsonInt64(char* out, size_t len, bool valid, int64_t value) {
    if (!valid) {
        snprintf(out, len, "null");
        return;
    }
    snprintf(out, len, "%lld", (long long)value);
}

static void jsonDouble(char* out, size_t len, bool valid, double value, int decimals) {
    if (!valid) {
        snprintf(out, len, "null");
        return;
    }

    switch (decimals) {
        case 3:
            snprintf(out, len, "%.3f", value);
            break;
        case 6:
            snprintf(out, len, "%.6f", value);
            break;
        default:
            snprintf(out, len, "%f", value);
            break;
    }
}

static void fillKalmanFields(LogEvent& e, const Kalman_State& state, double tick_us) {
    e.has_kalman = true;
    e.kalman_phi = state.phi;
    e.kalman_phi_us = state.phi * tick_us;
    e.kalman_f = state.f;
    e.kalman_p00 = state.p00;
    e.kalman_p01 = state.p01;
    e.kalman_p10 = state.p10;
    e.kalman_p11 = state.p11;
    e.kalman_y_phi = state.y0;
    e.kalman_k_phi = state.k00;
    e.kalman_k_f = state.k10;
}

void buildLogJson(const LogEvent& e, char* out, size_t len) {
    char pps_period_json[24];
    char ticks_json[24];
    char ticks_window_json[24];
    char ticks_raw_json[24];
    char seq_delta_json[24];
    char elapsed_us_json[24];
    char diff_json[32];
    char offset_ticks_json[32];
    char offset_us_json[32];
    char kalman_z_phi_json[32];
    char kalman_phi_json[32];
    char kalman_phi_us_json[32];
    char kalman_f_json[32];
    char kalman_p00_json[32];
    char kalman_p01_json[32];
    char kalman_p10_json[32];
    char kalman_p11_json[32];
    char kalman_y_phi_json[32];
    char kalman_k_phi_json[32];
    char kalman_k_f_json[32];
    char residual_us_json[32];

    jsonUInt32(pps_period_json, sizeof(pps_period_json), e.has_pps_period_us, e.pps_period_us);
    jsonUInt32(ticks_json, sizeof(ticks_json), e.has_ticks, e.ticks);
    jsonUInt32(ticks_window_json, sizeof(ticks_window_json), e.has_ticks_window, e.ticks_window);
    jsonUInt32(ticks_raw_json, sizeof(ticks_raw_json), e.has_ticks_raw, e.ticks_raw);
    jsonUInt32(seq_delta_json, sizeof(seq_delta_json), e.has_seq_delta, e.seq_delta);
    jsonUInt32(elapsed_us_json, sizeof(elapsed_us_json), e.has_elapsed_us, e.elapsed_us);
    jsonInt64(diff_json, sizeof(diff_json), e.has_diff, e.diff);
    jsonInt64(offset_ticks_json, sizeof(offset_ticks_json), e.has_offset, e.offset_ticks);
    jsonDouble(offset_us_json, sizeof(offset_us_json), e.has_offset, e.offset_us, 3);
    jsonDouble(kalman_z_phi_json, sizeof(kalman_z_phi_json), e.has_kalman_z_phi, e.kalman_z_phi, 6);
    jsonDouble(kalman_phi_json, sizeof(kalman_phi_json), e.has_kalman, e.kalman_phi, 6);
    jsonDouble(kalman_phi_us_json, sizeof(kalman_phi_us_json), e.has_kalman, e.kalman_phi_us, 3);
    jsonDouble(kalman_f_json, sizeof(kalman_f_json), e.has_kalman, e.kalman_f, 6);
    jsonDouble(kalman_p00_json, sizeof(kalman_p00_json), e.has_kalman, e.kalman_p00, 6);
    jsonDouble(kalman_p01_json, sizeof(kalman_p01_json), e.has_kalman, e.kalman_p01, 6);
    jsonDouble(kalman_p10_json, sizeof(kalman_p10_json), e.has_kalman, e.kalman_p10, 6);
    jsonDouble(kalman_p11_json, sizeof(kalman_p11_json), e.has_kalman, e.kalman_p11, 6);
    jsonDouble(kalman_y_phi_json, sizeof(kalman_y_phi_json), e.has_kalman, e.kalman_y_phi, 6);
    jsonDouble(kalman_k_phi_json, sizeof(kalman_k_phi_json), e.has_kalman, e.kalman_k_phi, 6);
    jsonDouble(kalman_k_f_json, sizeof(kalman_k_f_json), e.has_kalman, e.kalman_k_f, 6);
    jsonDouble(residual_us_json, sizeof(residual_us_json), e.has_residual, e.residual_us, 3);

    snprintf(
        out,
        len,
        "{\"id\":\"%s\"," 
        "\"aging\":%d,"
        "\"ms\":%lu,"
        "\"modo\":\"%s\","
        "\"first\":%s,"
        "\"pps_seq\":%lu,"
        "\"pps_pending\":%lu,"
        "\"pps_period_us\":%s,"
        "\"ms_since_last_pps\":%lu,"
        "\"holdover_steps\":%lu,"
        "\"offset_medido_disponible\":%s,"
        "\"kalman_prediccion\":%s,"
        "\"kalman_actualizado\":%s,"
        "\"ticks\":%s,"
        "\"ticks_window\":%s,"
        "\"ticks_raw\":%s,"
        "\"seq_delta\":%s,"
        "\"elapsed_us\":%s,"
        "\"interval_valid\":%s,"
        "\"diff_outlier\":%s,"
        "\"missed_pps\":%lu,"
        "\"diff\":%s,"
        "\"offset_ticks\":%s,"
        "\"offset_us\":%s,"
        "\"kalman_medicion\":\"%s\","
        "\"kalman_z_phi\":%s,"
        "\"kalman_phi\":%s,"
        "\"kalman_phi_us\":%s,"
        "\"kalman_f\":%s,"
        "\"kalman_p00\":%s,"
        "\"kalman_p01\":%s,"
        "\"kalman_p10\":%s,"
        "\"kalman_p11\":%s,"
        "\"kalman_y_phi\":%s,"
        "\"kalman_k_phi\":%s,"
        "\"kalman_k_f\":%s,"
        "\"residual_us\":%s%s}",
        e.id,
        e.aging,
        (unsigned long)e.ms,
        e.modo,
        jsonBool(e.first),
        (unsigned long)e.pps_seq,
        (unsigned long)e.pps_pending,
        pps_period_json,
        (unsigned long)e.ms_since_last_pps,
        (unsigned long)e.holdover_steps,
        jsonBool(e.offset_medido_disponible),
        jsonBool(e.kalman_prediccion),
        jsonBool(e.kalman_actualizado),
        ticks_json,
        ticks_window_json,
        ticks_raw_json,
        seq_delta_json,
        elapsed_us_json,
        jsonBool(e.interval_valid),
        jsonBool(e.diff_outlier),
        (unsigned long)e.missed_pps,
        diff_json,
        offset_ticks_json,
        offset_us_json,
        e.kalman_medicion,
        kalman_z_phi_json,
        kalman_phi_json,
        kalman_phi_us_json,
        kalman_f_json,
        kalman_p00_json,
        kalman_p01_json,
        kalman_p10_json,
        kalman_p11_json,
        kalman_y_phi_json,
        kalman_k_phi_json,
        kalman_k_f_json,
        residual_us_json,
        e.rtc_gnss_fields
    );
}

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
) {
    LogEvent e;
    e.id = device_id;
    e.aging = aging;
    e.ms = now_ms;
    e.modo = "HOLDOVER";
    e.first = false;
    e.pps_seq = seq_prev;
    e.pps_pending = 0;
    e.ms_since_last_pps = ms_since_last_pps;
    e.holdover_steps = holdover_steps;
    e.offset_medido_disponible = false;
    e.kalman_prediccion = true;
    e.kalman_actualizado = false;
    e.has_offset = true;
    e.offset_ticks = offset_ticks;
    e.offset_us = (double)offset_ticks * tick_us;
    e.kalman_medicion = "none";
    fillKalmanFields(e, kalman_state, tick_us);
    e.has_residual = false;
    e.rtc_gnss_fields = rtc_gnss_fields;

    buildLogJson(e, out, len);
}

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
) {
    LogEvent e;
    e.id = device_id;
    e.aging = aging;
    e.ms = now_ms;
    e.modo = "PPS_INICIAL";
    e.first = true;
    e.pps_seq = seq;
    e.pps_pending = pending;
    e.has_pps_period_us = true;
    e.pps_period_us = pps_period_us;
    e.offset_medido_disponible = false;
    e.kalman_prediccion = false;
    e.kalman_actualizado = false;
    e.has_ticks_raw = true;
    e.ticks_raw = ticks_raw;
    e.kalman_medicion = "none";
    e.rtc_gnss_fields = rtc_gnss_fields;

    buildLogJson(e, out, len);
}

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
) {
    const double offset_us = (double)offset_ticks * tick_us;
    const double kalman_phi_us = kalman_state.phi * tick_us;

    LogEvent e;
    e.id = device_id;
    e.aging = aging;
    e.ms = now_ms;
    e.modo = modo;
    e.first = false;
    e.pps_seq = seq;
    e.pps_pending = pending;
    e.has_pps_period_us = true;
    e.pps_period_us = pps_period_us;
    e.ms_since_last_pps = 0;
    e.holdover_steps = holdover_steps;
    e.offset_medido_disponible = offset_medido_disponible;
    e.kalman_prediccion = kalman_prediccion;
    e.kalman_actualizado = kalman_actualizado;

    e.has_ticks = offset_medido_disponible;
    e.ticks = ticks;
    e.has_ticks_window = true;
    e.ticks_window = ticks_window;
    e.has_ticks_raw = true;
    e.ticks_raw = ticks_raw;
    e.has_seq_delta = true;
    e.seq_delta = seq_delta;
    e.has_elapsed_us = true;
    e.elapsed_us = elapsed_us;

    e.interval_valid = interval_valid;
    e.diff_outlier = diff_outlier;
    e.missed_pps = missed_pps;
    e.has_diff = offset_medido_disponible;
    e.diff = diff;

    e.has_offset = true;
    e.offset_ticks = offset_ticks;
    e.offset_us = offset_us;

    e.kalman_medicion = kalman_actualizado ? "phi" : "none";
    e.has_kalman_z_phi = kalman_actualizado;
    e.kalman_z_phi = (double)offset_ticks;
    fillKalmanFields(e, kalman_state, tick_us);

    e.has_residual = offset_medido_disponible;
    e.residual_us = offset_us - kalman_phi_us;
    e.rtc_gnss_fields = rtc_gnss_fields;

    buildLogJson(e, out, len);
}