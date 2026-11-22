#include <Arduino.h>
#include <RTClib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "Setup.h"
#include "Json_Log_Builder.h"
#include "Physical_Event_Module.h"

Setup app("prototipo-2");
PhysicalEventModule physical_event(app.pcnt());
static bool rtc_sincronizado_utc = false;
static uint32_t rtc_sync_pps_seq = 0;
//static uint32_t rtc_sync_inicio_post_pps_us = 0;
//static uint32_t estadistica_inicio_pps_seq = 0;
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
    if (inicio_post_pps_us > Setup::RTC_SYNC_MAX_POST_PPS_US) return;

    if (app.rtc().setFromDateTime(gps_local)) {
        app.clockRtc().invalidateOnCalendarWrite();
        rtc_sincronizado_utc = true;
        rtc_sync_pps_seq = seq;
        //rtc_sync_inicio_post_pps_us = inicio_post_pps_us;
    }
}

static void runBackgroundTasks() {
    static uint32_t last_ms = 0;
    const uint32_t now_ms = millis();
    if (uint32_t(now_ms - last_ms) < Setup::BACKGROUND_TASKS_PERIOD_MS) return;
    last_ms = now_ms;
    app.gps().update();
    app.gps().consultarUbxUtcSiCorresponde();
}

static void addGpsTelemetry(JsonLogRecord& record) {
    record.gps_sats = app.gps().getGgaNumSats();
    record.gps_pdop = app.gps().getGsaPdop();
    record.gps_hdop = app.gps().getGgaHdop();
    record.has_utc_validation = true;
    record.utc_valid = utcListaParaInicializar();
}

// Usa una sola muestra de ticks para los TRES relojes.
// Para HOLDOVER el GNSS se deja en null; para EVENTO_FISICO se interpola
// desde el ultimo PPS validado usando el contador de 32k.
static void sendRecord(JsonLogRecord& record, uint64_t sample_ticks,
                       bool pps_comparable, bool allow_kalman) {
    record.clocks.has_ticks = true;
    record.clocks.ticks = sample_ticks;
    record.clocks.gnss_ok = pps_comparable &&
        app.clockGnss().getTimeAtTicks(sample_ticks, record.clocks.gnss_us);
    record.clocks.rtc_raw_ok =
        app.clockRtc().getRawTimeAtTicks(sample_ticks, record.clocks.rtc_raw_us);
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
    r.rtc_calibrated = app.clockRtc().calibrated();
    r.rtc_initial_correction_us = app.clockRtc().initialCorrectionUs();
    return r;
}

// La fase GNSS-RTC se fija UNA VEZ. No toca RTC I2C, SQW ni el filtro.
static void calibrarRtcInicialEnPps(uint64_t ticks, bool pps_gnss_valido) {
    if (!pps_gnss_valido || app.clockRtc().calibrated()) return;
    int64_t gnss_us = 0;
    if (!app.clockGnss().getTimeAtTicks(ticks, gnss_us)) return;
    if (!app.clockRtc().calibrateOnce(ticks, gnss_us)) return;
    // El reloj Kalman parte de ESTE mismo origen, sin reiniciar ni entrenar
    // el filtro. Asi la correccion de deriva vale cero al calibrar.
    app.clockKalman().setReference();
    Serial.printf("RTC calibrado: correccion inicial %+lld us\n",
        static_cast<long long>(app.clockRtc().initialCorrectionUs()));
}

// El instante es el de la ISR. Las tres horas se calculan DESPUES, usando
// los mismos ticks, sin consultar el contador tres veces ni entrenar Kalman.
static void publicarEventosFisicos() {
    PhysicalEventSample event;
    for (int i = 0; i < 4 && physical_event.consume(event); ++i) {
        JsonLogRecord record = recordBase(JsonLogType::EVENTO_FISICO, millis());
        physical_event.fillRecord(record, event);
        // Estado disponible al procesar el evento (la ISR NO lee Kalman).
        // No se ejecutan predict() ni updatePhi() por el pulsador.
        record.has_kalman = app.clockRtc().calibrated();
        if (record.has_kalman) record.kalman = app.kalman().getState();
        // Referencia PPS anterior al boton, si aun esta disponible.
        TickEdgeSample last_pps;
        if (app.pps().peek(last_pps)) {
            physical_event.fillPpsAge(record, event, last_pps);
        }
        // GNSS al evento: interpolado desde el ultimo PPS validado por PCNT.
        // Si no hay referencia, ClockGNSS devuelve false y se publica null.
        sendRecord(record, event.ticks, true, true);
    }
}

void setup() {
    app.begin();
    if (!physical_event.begin(Setup::PIN_PULSADOR)) {
        Serial.println("ERROR: no se pudo iniciar pulsador GPIO13");
        while (true) delay(1000);
    }
    Serial.println("main_kalman: entrenamiento PPS original + tres clases Clock");
    Serial.println("ClockRTC = CALENDARIO DS3231 + SQW + PCNT 32 kHz");
}

void loop() {
    // El evento se procesa ANTES de actualizar SQW o Kalman/PPS, para
    // evitar consultar un anclaje o estado posteriores a su captura.
    publicarEventosFisicos();

    // La lectura I2C del SQW se atiende fuera de ISR, incluso sin GNSS.
    app.clockRtc().update();

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
            ms_since_last_pps > Setup::HOLDOVER_TIMEOUT_MS;
        const bool corresponde_predecir =
            now_ms - last_holdover_predict_ms >= Setup::HOLDOVER_STEP_MS;

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
        timing_error_us <= Setup::PPS_TIME_TOLERANCE_US;

    // if (interval_valid && seq_delta == 1 && !venia_de_holdover) {
    //     const int64_t diff_candidato = int64_t(ticks_window) - 32768LL;
    //     diff_outlier = (diff_candidato > MAX_DIFF_TICKS) ||
    //                    (diff_candidato < -MAX_DIFF_TICKS);
    // }
    const bool pps_apto_para_inicio =
        pending == 1 && seq_delta == 1 && interval_valid &&
        !venia_de_holdover;
    const bool sincronizado_antes = rtc_sincronizado_utc;
    sincronizarRtcConUtcEnPps(seq, pps_time_us, pps_apto_para_inicio);

    if (!sincronizado_antes) {
        if (rtc_sincronizado_utc) {
            offset_ticks = 0;
            app.kalman().reset();
            kalman_entrenado = false;
            // Kalman solo fija su origen horario cuando RTC y GNSS
            // puedan compararse en el MISMO PPS (no en la escritura I2C).
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
        const bool gnss_inicio_valido = app.clockGnss().observePps(
            pps_apto, seq, ticks_raw,
            rtc_sincronizado_utc ? &estado_inicial : nullptr,
            rmc_recent, gnss_local, rtc_sincronizado_utc,
            rtc_sync_pps_seq, utcListaParaInicializar());
        calibrarRtcInicialEnPps(ticks_raw, gnss_inicio_valido);
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
    int64_t diff = 0;
    bool diff_outlier = false;
    if (interval_valid && !venia_de_holdover) {
        const int64_t expected_ticks = int64_t(32768) * int64_t(seq_delta);
        diff = int64_t(ticks_window) - expected_ticks;
        diff_outlier = (diff > Setup::MAX_DIFF_TICKS) ||
                       (diff < -Setup::MAX_DIFF_TICKS);
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
        // if (estadistica_inicio_pps_seq == 0)
        //     estadistica_inicio_pps_seq = seq;
    } else {
        kalman_state = app.kalman().getState();
        kalman_actualizado = false;
        kalman_prediccion = false;
    }

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
    const bool gnss_valido_en_pps = app.clockGnss().observePps(
        pps_apto_para_relojes, seq, ticks_raw, &kalman_state,
        rmc_recent, gnss_local, rtc_sincronizado_utc,
        rtc_sync_pps_seq, utcListaParaInicializar());
    calibrarRtcInicialEnPps(ticks_raw, gnss_valido_en_pps);

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