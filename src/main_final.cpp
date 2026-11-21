#include <Arduino.h>
#include <RTClib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "Setup.h"
#include "Json_Log_Builder.h"

// Basado en main_kalman.cpp del ZIP del 20/11.
// Se conserva EL ALGORITMO de entrenamiento y holdover de Kalman.
// Three_Clocks_Module se reemplaza por ClockGNSS + ClockRTC + ClockKalman.

static constexpr uint32_t PPS_TIME_TOLERANCE_US = 200000UL;
static constexpr int64_t MAX_DIFF_TICKS = 10;
static constexpr uint32_t HOLDOVER_TIMEOUT_MS = 1500;
static constexpr uint32_t HOLDOVER_STEP_MS = 1000;
static constexpr uint32_t BACKGROUND_TASKS_PERIOD_MS = 20;
static constexpr uint32_t RTC_SYNC_MAX_POST_PPS_US = 50000;

Setup app("esp32-prototipo-1");
static bool rtc_sincronizado_utc = false;
static uint32_t rtc_sync_pps_seq = 0;
static uint32_t rtc_sync_inicio_post_pps_us = 0;
static uint32_t estadistica_inicio_pps_seq = 0;
static bool kalman_entrenado = false;

static bool utcListaParaInicializar(DateTime* rmc_out = nullptr) {
    GpsUtcSample sample;
    if (!app.gps().getValidatedUtcSample(sample, 3000, 1500)) return false;
    if (rmc_out) *rmc_out = DateTime(uint32_t(sample.local_epoch_s));
    return true;
}

static void sincronizarRtcConUtcEnPps(uint32_t seq, uint32_t pps_us,
                                     bool pps_apto) {
    if (rtc_sincronizado_utc || !pps_apto) return;
    DateTime gps_local(2000, 1, 1, 0, 0, 0);
    if (!utcListaParaInicializar(&gps_local)) return;

    const uint32_t inicio_post_pps_us = uint32_t(micros() - pps_us);
    if (inicio_post_pps_us > RTC_SYNC_MAX_POST_PPS_US) return;

    if (app.rtc().setFromDateTime(gps_local)) {
        app.clockRtc().invalidateOnCalendarWrite();
        rtc_sincronizado_utc = true;
        rtc_sync_pps_seq = seq;
        rtc_sync_inicio_post_pps_us = inicio_post_pps_us;
    }
}

static void runBackgroundTasks() {
    static uint32_t last_ms = 0;
    const uint32_t now_ms = millis();
    if (uint32_t(now_ms - last_ms) < BACKGROUND_TASKS_PERIOD_MS) return;
    last_ms = now_ms;
    app.gps().update();
    static uint32_t ultimo_diag = 0;

if (millis() - ultimo_diag >= 5000) {
    ultimo_diag = millis();

    TickEdgeSample pps;
    bool recibido = app.pps().peek(pps);
    auto stats = app.telemetryQueue().getStats();

    Serial.printf(
        "[DIAG] PPS=%lu | RTC=%s | Cola=%lu | Enviados=%lu | Offline=%lu | Fallidos=%lu\n",
        (unsigned long)(recibido ? pps.seq : 0),
        app.clockRtc().lastState(),
        (unsigned long)stats.enqueued,
        (unsigned long)stats.tcp_accepted,
        (unsigned long)stats.tcp_offline,
        (unsigned long)stats.tcp_write_failed
    );
}
    app.gps().consultarUbxUtcSiCorresponde();
}

static void addGpsTelemetry(JsonLogRecord& record) {
    record.gps_sats = app.gps().getGgaNumSats();
    record.gps_pdop = app.gps().getGsaPdop();
    record.gps_hdop = app.gps().getGgaHdop();
    record.has_utc_validation = true;
    record.utc_valid = utcListaParaInicializar();
}

// Conservar las tres fechas ISO que publicaba main_final.cpp del ZIP.
// La fuente de cada hora ahora es la definida en su clase Clock.
static void formatTimeJson(char* out, size_t size, bool ok, int64_t local_us) {
    if (!ok || local_us <= 0) {
        snprintf(out, size, "null");
        return;
    }
    const uint32_t secs = uint32_t(local_us / 1000000LL);
    const uint32_t us = uint32_t(local_us % 1000000LL);
    const DateTime dt(secs);
    snprintf(out, size,
        "\"%04u-%02u-%02uT%02u:%02u:%02u.%06lu\"",
        unsigned(dt.year()), unsigned(dt.month()), unsigned(dt.day()),
        unsigned(dt.hour()), unsigned(dt.minute()), unsigned(dt.second()),
        (unsigned long)us);
}

// Usa una sola muestra de ticks para los TRES relojes.
// Para GNSS fuera de PPS se devuelve null: no inventa nueva UTC.
static void sendRecord(JsonLogRecord& record, uint64_t sample_ticks,
                       bool pps_comparable, bool allow_kalman) {
    record.clocks.has_ticks = true;
    record.clocks.ticks = sample_ticks;
    record.clocks.gnss_ok = pps_comparable &&
        app.clockGnss().getTimeAtTicks(sample_ticks, record.clocks.gnss_us);
    record.clocks.rtc_ok =
        app.clockRtc().getTimeAtTicks(sample_ticks, record.clocks.rtc_us);
    record.clocks.kalman_ok = allow_kalman &&
        app.clockKalman().getTimeAtTicks(sample_ticks, record.clocks.kalman_us);
    addGpsTelemetry(record);
    char json[Telemetry_Queue_Module::JSON_CAPACITY];
    if (!buildLogJson(record, json, sizeof(json))) {
        Serial.println("ERROR: no se pudo construir JSON del registro");
        return;
    }
    char gnss_json[40], rtc_json[40], kalman_json[40];
    formatTimeJson(gnss_json, sizeof(gnss_json),
                   record.clocks.gnss_ok, record.clocks.gnss_us);
    formatTimeJson(rtc_json, sizeof(rtc_json),
                   record.clocks.rtc_ok, record.clocks.rtc_us);
    formatTimeJson(kalman_json, sizeof(kalman_json),
                   record.clocks.kalman_ok, record.clocks.kalman_us);
    const size_t used = strlen(json);
    if (used < 2 || json[used-1] != '}') return;
    const size_t available = sizeof(json) - (used-1);
    const int n = snprintf(json + used - 1, available,
        ",\"gnss\":%s,\"rtc\":%s,\"kalman\":%s}",
        gnss_json, rtc_json, kalman_json);
    if (n <= 0 || size_t(n) >= available) return;
    app.telemetryQueue().enqueue(json);
}

static JsonLogRecord recordBase(JsonLogType type, uint32_t ms) {
    JsonLogRecord r{};
    r.id = app.deviceId();
    r.tipo = type;
    r.ms = ms;
    r.aging = app.agingApplied();
    r.rtc_aligned = rtc_sincronizado_utc;
    r.kalman_trained = kalman_entrenado;
    return r;
}

void setup() {
    app.begin();
    Serial.println("main_kalman: entrenamiento PPS original + tres clases Clock");
    Serial.println("ClockRTC = CALENDARIO DS3231 + SQW + PCNT 32 kHz");
}

void loop() {
    // La lectura I2C del SQW se atiende fuera de ISR, incluso sin GNSS.
    app.clockRtc().update();

    // Estado preservado de main_kalman.cpp.
    static bool hay_anterior = false;
    static uint64_t ticks_raw_prev = 0;
    static uint32_t seq_prev = 0;
    static uint32_t pps_time_us_prev = 0;
    static int64_t offset_ticks = 0;
    static uint32_t last_pps_seen_ms = 0;
    static uint32_t last_holdover_predict_ms = 0;
    static uint32_t holdover_steps = 0;
    static bool en_holdover = false;

    TickEdgeSample muestra_pps;
    const bool hay_muestra = app.pps().consume(muestra_pps);

    // ============================================================
    // CASO 1: NO LLEGÓ PPS NUEVO (mismos predictores y temporización).
    // ============================================================
    if (!hay_muestra) {
        runBackgroundTasks();
        if (!hay_anterior || !rtc_sincronizado_utc) return;

        const uint32_t now_ms = millis();
        const uint32_t ms_since_last_pps = now_ms - last_pps_seen_ms;
        const bool pps_perdido =
            ms_since_last_pps > HOLDOVER_TIMEOUT_MS;
        const bool corresponde_predecir =
            now_ms - last_holdover_predict_ms >= HOLDOVER_STEP_MS;

        if (pps_perdido && corresponde_predecir) {
            en_holdover = true;
            holdover_steps++;
            app.kalman().predict(); // IGUAL a main_kalman.cpp
            Kalman_State state = app.kalman().getState();

            JsonLogRecord record = recordBase(JsonLogType::HOLDOVER, now_ms);
            record.detalle = "HOLDOVER_PREDICHO";
            record.has_offset_ticks = true;
            record.offset_ticks = offset_ticks;
            record.kalman_prediccion = true;
            record.kalman_actualizado = false;
            record.has_kalman = true;
            record.kalman = state;
            record.ms_since_last_pps = ms_since_last_pps;
            record.holdover_steps = holdover_steps;
            sendRecord(record, app.pcnt().read(), false, true);
            last_holdover_predict_ms = now_ms;
        }
        return;
    }

    // ============================================================
    // CASO 2: LLEGÓ PPS NUEVO
    // ============================================================
    const uint64_t ticks_raw = muestra_pps.ticks;
    const uint32_t seq = muestra_pps.seq;
    const uint32_t pending = muestra_pps.pending;
    const uint32_t pps_time_us = muestra_pps.at_us;
    const uint32_t now_ms = millis();
    const bool venia_de_holdover = en_holdover;
    const uint32_t holdover_steps_previos = holdover_steps;
    last_pps_seen_ms = now_ms;
    last_holdover_predict_ms = now_ms;

    if (!hay_anterior) {
        ticks_raw_prev = ticks_raw;
        seq_prev = seq;
        pps_time_us_prev = pps_time_us;
        hay_anterior = true;
        JsonLogRecord record = recordBase(JsonLogType::PPS, now_ms);
        record.detalle = "PPS_INICIAL";
        record.has_pps = true;
        record.pps = muestra_pps;
        record.interval_valid = false;
        sendRecord(record, ticks_raw, false, false);
        return;
    }

    const uint32_t ticks_window = uint32_t(ticks_raw - ticks_raw_prev);
    const uint32_t seq_delta = seq - seq_prev;
    const uint32_t elapsed_us = pps_time_us - pps_time_us_prev;
    const uint64_t expected_us = uint64_t(seq_delta) * 1000000ULL;
    const uint64_t timing_error_us =
        (elapsed_us >= expected_us)
            ? uint64_t(elapsed_us) - expected_us
            : expected_us - uint64_t(elapsed_us);
    const bool interval_valid = seq_delta > 0 &&
        timing_error_us <= PPS_TIME_TOLERANCE_US;

    int64_t diff = 0;
    bool diff_outlier = false;
    if (interval_valid && seq_delta == 1 && !venia_de_holdover) {
        const int64_t diff_candidato = int64_t(ticks_window) - 32768LL;
        diff_outlier = (diff_candidato > MAX_DIFF_TICKS) ||
                       (diff_candidato < -MAX_DIFF_TICKS);
    }
    const bool pps_apto_para_inicio =
        pending == 1 && seq_delta == 1 && interval_valid &&
        !venia_de_holdover;
    const bool sincronizado_antes = rtc_sincronizado_utc;
    sincronizarRtcConUtcEnPps(seq, pps_time_us, pps_apto_para_inicio);

    if (!sincronizado_antes) {
        if (rtc_sincronizado_utc) {
            offset_ticks = 0;
            app.kalman().reset();  // IGUAL a main_kalman
            kalman_entrenado = false;
            app.clockKalman().setReference();
            en_holdover = false;
            holdover_steps = 0;
        }
        const Kalman_State estado_inicial = app.kalman().getState();
        DateTime gnss_local(2000, 1, 1, 0, 0, 0);
        const bool rmc_recent =
            app.gps().hasValidRmcRecent(3000) &&
            app.gps().getLastValidLocalDateTime(gnss_local) &&
            app.gps().hasValidRmcRecent(1500);
        const bool pps_apto = rtc_sincronizado_utc && pps_apto_para_inicio;
        app.clockGnss().observePps(pps_apto, seq, ticks_raw,
            rtc_sincronizado_utc ? &estado_inicial : nullptr,
            rmc_recent, gnss_local, rtc_sincronizado_utc,
            rtc_sync_pps_seq, utcListaParaInicializar());
        if (rtc_sincronizado_utc) {
            app.clockKalman().captureStateAtTicks(ticks_raw);
        }
        JsonLogRecord record = recordBase(JsonLogType::PPS, now_ms);
        record.detalle = rtc_sincronizado_utc
            ? "PPS_ORIGEN_UTC" : "PPS_ESPERANDO_UTC";
        record.has_pps = true;
        record.pps = muestra_pps;
        record.interval_valid = interval_valid;
        record.has_kalman = rtc_sincronizado_utc;
        record.kalman = estado_inicial;
        record.has_offset_ticks = rtc_sincronizado_utc;
        record.offset_ticks = offset_ticks;
        sendRecord(record, ticks_raw, pps_apto, rtc_sincronizado_utc);
        ticks_raw_prev = ticks_raw;
        seq_prev = seq;
        pps_time_us_prev = pps_time_us;
        return;
    }

    // ==================== KALMAN: SIN CAMBIOS ====================
    if (interval_valid && !venia_de_holdover) {
        const int64_t expected_ticks = int64_t(32768) * int64_t(seq_delta);
        diff = int64_t(ticks_window) - expected_ticks;
        diff_outlier = (diff > MAX_DIFF_TICKS) ||
                       (diff < -MAX_DIFF_TICKS);
        if (!diff_outlier) offset_ticks += diff;
    }

    bool kalman_actualizado = false;
    bool kalman_prediccion = false;
    Kalman_State kalman_state;
    if (interval_valid && seq_delta == 1 && !diff_outlier &&
        !venia_de_holdover) {
        app.kalman().predict();
        kalman_prediccion = true;
        app.kalman().updatePhi(double(offset_ticks));
        kalman_state = app.kalman().getState();
        kalman_actualizado = true;
        kalman_entrenado = true;
        if (estadistica_inicio_pps_seq == 0)
            estadistica_inicio_pps_seq = seq;
    } else {
        kalman_state = app.kalman().getState();
        kalman_actualizado = false;
        kalman_prediccion = false;
    }

    // Misma reconstrucción de segundos y controles de Three_Clocks_Module,
    // pero sólo aplica a la referencia GNSS. NO define la hora RTC.
    app.clockGnss().observePpsInterval(
        venia_de_holdover, elapsed_us, seq_delta, pending, ticks_window);
    DateTime gnss_local(2000, 1, 1, 0, 0, 0);
    const bool rmc_recent =
        app.gps().hasValidRmcRecent(3000) &&
        app.gps().getLastValidLocalDateTime(gnss_local) &&
        app.gps().hasValidRmcRecent(1500);
    const bool pps_apto_para_relojes =
        pending == 1 && seq_delta == 1 &&
        kalman_actualizado && !venia_de_holdover;
    app.clockGnss().observePps(
        pps_apto_para_relojes, seq, ticks_raw, &kalman_state,
        rmc_recent, gnss_local, rtc_sincronizado_utc,
        rtc_sync_pps_seq, utcListaParaInicializar());

    // Estado Kalman del mismo PPS; la lectura posterior no lo reentrena.
    app.clockKalman().captureStateAtTicks(ticks_raw);
    const char* modo = venia_de_holdover ? "PPS_REENGANCHE" :
        (kalman_actualizado ? "PPS_KALMAN_Z_PHI" : "PPS_SIN_UPDATE_KALMAN");
    JsonLogRecord record = recordBase(JsonLogType::PPS, now_ms);
    record.detalle = modo;
    record.has_pps = true;
    record.pps = muestra_pps;
    record.interval_valid = interval_valid;
    record.has_offset_ticks =
        interval_valid && !diff_outlier && !venia_de_holdover;
    record.offset_ticks = offset_ticks;
    record.kalman_prediccion = kalman_prediccion;
    record.kalman_actualizado = kalman_actualizado;
    record.has_kalman = true;
    record.kalman = kalman_state;
    if (venia_de_holdover) record.holdover_steps = holdover_steps_previos;
    sendRecord(record, ticks_raw, pps_apto_para_relojes, true);

    ticks_raw_prev = ticks_raw;
    seq_prev = seq;
    pps_time_us_prev = pps_time_us;
    if (venia_de_holdover) {
        en_holdover = false;
        holdover_steps = 0;
    }
}
