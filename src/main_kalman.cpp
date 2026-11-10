#include <Arduino.h>
#include <string.h>
#include "RTC32K_Pcnt_Capture_Module.h"
#include "Kalman_Filter.h"
#include "GPS_Module.h"
#include "RTC_Module.h"
#include "WiFi_Module.h"
#include "Telemetry_Queue_Module.h"
#include "Json_Log_Builder.h"
#include "Three_Clocks_Module.h"
#include "RTC_GNSS_Diagnostics_Module.h"
#include "Physical_Event_Module.h"
#include "RTC_SQW_Clock_Module.h"

// ==================== PINES ====================

#define PIN_32K_RTC 18
#define PIN_PPS_GPS 27
#define PIN_SQW_RTC 19
#define PIN_GPS_RX 16
#define PIN_GPS_TX 17
#define PIN_PULSADOR 13
#define GPS_BAUD 9600

GPS_Module gps(Serial2, PIN_GPS_RX, PIN_GPS_TX, PIN_PPS_GPS, GPS_BAUD);
RTC_Module rtc;

// ==================== PCNT ====================
#define PPS_TIME_TOLERANCE_US 200000UL

static constexpr int64_t MAX_DIFF_TICKS = 10;
static constexpr double TICK_US = 1000000.0 / 32768.0;

// ==================== HOLDOVER ====================
// Indica cuánto tiempo sin PPS, se considera pérdida de referencia.
static constexpr uint32_t HOLDOVER_TIMEOUT_MS = 1500;
// Cada cuánto se hace una predicción en holdover.
static constexpr uint32_t HOLDOVER_STEP_MS = 1000;

// ==================== AGING DS3231 ====================
static constexpr int8_t AGING_VALUE = -13;
int8_t aging_aplicado = 0;

// ==================== WIFI ====================
const char* WIFI_SSID = "PATAN";
const char* WIFI_PASS = "autoslocos";
const char* SERVER_IP = "192.168.0.119";
const uint16_t SERVER_PORT = 8080;
const char* DEVICE_ID = "esp32-prototipo-1";

WiFi_Module wifiModule(
    WIFI_SSID,
    WIFI_PASS,
    SERVER_IP,
    SERVER_PORT,
    DEVICE_ID
);
Telemetry_Queue_Module telemetryQueue(wifiModule);

// ==================== KALMAN ====================
Kalman_Filter kalman;

// ==================== FECHA/HORA GNSS + RTC ====================
static constexpr uint32_t BACKGROUND_TASKS_PERIOD_MS = 20;
Three_Clocks_Module threeClocks;
RTC_GNSS_Diagnostics_Module rtcGnssDiagnostics(gps, rtc, threeClocks);

// ==================== INICIALIZACION DE HORA ====================
// Exigimos RMC A + checksum (controlado por GPS_Module) y UBX-NAV-TIMEUTC
// reciente, con validTOW, validWKN y validUTC activos.
static constexpr uint32_t UTC_UBX_MAX_AGE_MS = 3000;
static constexpr uint32_t RTC_SYNC_MAX_POST_PPS_US = 50000;
static bool rtc_sincronizado_utc = false;
static uint32_t rtc_sync_pps_seq = 0;
static uint32_t rtc_sync_inicio_post_pps_us = 0;
// Primer intervalo PPS utilizado para estadisticas; nunca antes de UTC validada.
static uint32_t estadistica_inicio_pps_seq = 0;

static bool utcListaParaInicializar(DateTime* rmc_out = nullptr) {
    const UbxUtcDiagnostics u = gps.getUbxUtcDiagnostics();
    if (!u.recibido || !u.fecha_razonable ||
        (u.flags & 0x07) != 0x07 ||
        (uint32_t)(millis() - u.recibido_ms) > UTC_UBX_MAX_AGE_MS ||
        !gps.hasValidRmcRecent(1500)) {
        return false;
    }
    DateTime rmc_local(2000, 1, 1, 0, 0, 0);
    if (!gps.getLastValidLocalDateTime(rmc_local)) return false;

    // No aceptar dos escalas UTC que discrepen por varios segundos.
    const DateTime ubx_utc(u.year, u.month, u.day, u.hour, u.minute, u.second);
    const int64_t diferencia_s =
        ((int64_t)rmc_local.unixtime() + 3LL * 3600LL) -
        (int64_t)ubx_utc.unixtime();
    if (diferencia_s != 0 && diferencia_s != 1) return false;
    if (rmc_out != nullptr) *rmc_out = rmc_local;
    return true;
}


// Se llama una sola vez, DESPUES de capturar un PPS y FUERA de la ISR.
// La escritura I2C arranca cerca de ese PPS: no supone que la fase del
// calendario DS3231 quede alineada a microsegundos con el 1PPS.
static void sincronizarRtcConUtcEnPps(uint32_t seq, uint32_t pps_us,
                                      bool pps_apto) {
    if (rtc_sincronizado_utc || !pps_apto) return;
    DateTime gps_local(2000, 1, 1, 0, 0, 0);
    if (!utcListaParaInicializar(&gps_local)) return;

    const uint32_t inicio_post_pps_us = (uint32_t)(micros() - pps_us);
    // Si el loop atendio tarde el PPS, no ajustamos el calendario sobre
    // una etiqueta temporal que ya pudo haber cambiado de segundo.
    if (inicio_post_pps_us > RTC_SYNC_MAX_POST_PPS_US) return;

    if (rtc.setFromDateTime(gps_local)) {
        RTC_SQW_Clock::invalidateOnCalendarWrite();
        rtc_sincronizado_utc = true;
        rtc_sync_pps_seq = seq;
        rtc_sync_inicio_post_pps_us = inicio_post_pps_us;
    }
}

// Decisiones de inicializacion y las referencias, el armado y los diagnosticos JSON se encuentran en RTC_GNSS_Diagnostics_Module.
static void buildRtcGnssJsonFields(
    char* out, size_t len, bool pps_apto = false,
    uint32_t seq = 0, uint32_t ticks_raw = 0,
    const Kalman_State* state = nullptr,
    bool holdover_estimated = false) {
    RTC_GNSS_InitState init;
    init.utc_reference_ready = utcListaParaInicializar();
    init.rtc_synced_utc = rtc_sincronizado_utc;
    init.rtc_sync_pps_seq = rtc_sync_pps_seq;
    init.rtc_sync_start_delay_us = rtc_sync_inicio_post_pps_us;
    init.statistics_start_pps_seq = estadistica_inicio_pps_seq;
    rtcGnssDiagnostics.buildJsonFields(out, len, init,
                                        pps_apto, seq, ticks_raw, state,
                                        holdover_estimated);
    // Cuarto reloj
    RTC_SQW_Clock::appendClockJson(out, len,
                                  seq != 0 || holdover_estimated, ticks_raw);
}


void runBackgroundTasks() {
    static uint32_t last_background_ms = 0;

    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - last_background_ms) < BACKGROUND_TASKS_PERIOD_MS) {
        return;
    }

    last_background_ms = now_ms;

    gps.update();
    gps.consultarUbxUtcSiCorresponde();
    rtcGnssDiagnostics.observeRmcAfterPps();
}

void setup() {
    Serial.begin(115200);
    delay(300);
    if (!rtc.begin()) {
        Serial.println("ERROR: no se detectó el DS3231 por I2C");
        while (true) {
            delay(1000);
        }
    }

    gps.beginNmeaOnly();
    Serial.println("GNSS NMEA iniciado por GPS_Module sobre Serial2");
    Serial.print("GPS RX GPIO: ");
    Serial.println(PIN_GPS_RX);

    int8_t aging_anterior = 0;
    if (!rtc.readAging(aging_anterior)) {
        Serial.println("ERROR: no se pudo leer el aging del DS3231");
        while (true) {
            delay(1000);
        }
    }

    Serial.printf(
        "Aging anterior: %d | Aging solicitado: %d\n",
        (int)aging_anterior,
        (int)AGING_VALUE
    );

    if (!rtc.applyAgingValue(AGING_VALUE)) {
        Serial.println("ERROR: no se pudo aplicar/verificar el aging");
        while (true) {
            delay(1000);
        }
    }

    aging_aplicado = rtc.getAppliedAging();
    Serial.printf("Aging aplicado y verificado: %d\n", (int)aging_aplicado);
    RTC32K_Pcnt_Capture::begin(PIN_32K_RTC, PIN_PPS_GPS);
    if (!RTC_SQW_Clock::begin(PIN_SQW_RTC)) {
        Serial.println("ADVERTENCIA: RTC/SQW no disponible; reloj_rtc_calendario_sqw sera null");
    }
    if (!telemetryQueue.begin()) {
        Serial.println("ERROR: no se pudo crear la cola/tarea de telemetria");
        while (true) {
            delay(1000);
        }
    }
    Serial.println("PCNT iniciado. Telemetria en CORE 0. Esperando PPS...");
    // Inicializacion del pulsador compartido.
    Physical_Event::begin(PIN_PULSADOR);

    Serial.println("Evento fisico iniciado en GPIO13");
}

// ==================== MEDICIÓN ====================
void loop() {
    // Primero se atiende PPS/PCNT. GNSS queda en background; los JSON van por WiFi en CORE 0.
    static bool hay_anterior = false;
    static uint32_t ticks_raw_prev = 0;
    static uint32_t seq_prev = 0;
    static uint32_t pps_time_us_prev = 0;
    static int64_t offset_ticks = 0;
    static uint32_t last_pps_seen_ms = 0;
    static uint32_t last_holdover_predict_ms = 0;
    static uint32_t holdover_steps = 0;
    static bool en_holdover = false;
    RTC32K_Pcnt_Capture::PpsSample muestra_pps;
    const bool hay_muestra = RTC32K_Pcnt_Capture::consumePps(muestra_pps);
    const uint32_t ticks_raw = muestra_pps.ticks_raw;
    const uint32_t seq = muestra_pps.seq;
    const uint32_t pending = muestra_pps.pending;
    const uint32_t pps_time_us = muestra_pps.pps_time_us;
    const uint32_t pps_period_us = muestra_pps.pps_period_us;
    // ============================================================
    // CASO 1: NO LLEGÓ PPS NUEVO
    // ============================================================
    if (!hay_muestra) {
        // Atender SQW aun cuando no llegue GNSS/PPS (tambien en holdover).
        RTC_SQW_Clock::service(rtc);
        runBackgroundTasks();

        // Publicar los eventos del pulsador capturados
         Physical_Event::publishPending(
            DEVICE_ID,
            telemetryQueue
        );


        // Antes de UTC valida/RTC sincronizado no hay filtro que predecir:
        // tampoco se registra HOLDOVER.
        if (!hay_anterior || !rtc_sincronizado_utc) {
            return;
        }
        const uint32_t now_ms = millis();
        const uint32_t ms_since_last_pps = now_ms - last_pps_seen_ms;
        const bool pps_perdido =
            ms_since_last_pps > HOLDOVER_TIMEOUT_MS;
        const bool corresponde_predecir =
            now_ms - last_holdover_predict_ms >= HOLDOVER_STEP_MS;
        if (pps_perdido && corresponde_predecir) {
            en_holdover = true;
            holdover_steps++;

            // Solo se aplica la etapa de predicción:
            kalman.predict();
            Kalman_State kalman_state = kalman.getState();
            char rtc_gnss_fields[1024];
            uint32_t ticks_actuales = 0;
            const bool pcnt_ok =
                RTC32K_Pcnt_Capture::snapshotCurrentTicks(ticks_actuales);
            buildRtcGnssJsonFields(
                rtc_gnss_fields, sizeof(rtc_gnss_fields),
                false, 0, ticks_actuales,
                pcnt_ok ? &kalman_state : nullptr, pcnt_ok);

            char json[1800];
            buildHoldoverLogJson(
                json,
                sizeof(json),
                DEVICE_ID,
                (int)aging_aplicado,
                now_ms,
                seq_prev,
                ms_since_last_pps,
                holdover_steps,
                offset_ticks,
                TICK_US,
                kalman_state,
                rtc_gnss_fields
            );
            telemetryQueue.enqueue(json);
            last_holdover_predict_ms = now_ms;
        }
        return;
    }
    // ============================================================
    // CASO 2: LLEGÓ PPS NUEVO
    // ============================================================
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
        last_pps_seen_ms = now_ms;
        last_holdover_predict_ms = now_ms;
        char rtc_gnss_fields[1024];
        buildRtcGnssJsonFields(rtc_gnss_fields, sizeof(rtc_gnss_fields));

        char json[1800];
        buildInitialPpsLogJson(
            json,
            sizeof(json),
            DEVICE_ID,
            (int)aging_aplicado,
            now_ms,
            seq,
            pending,
            pps_period_us,
            ticks_raw,
            rtc_gnss_fields
        );
        telemetryQueue.enqueue(json);
        return;
    }
    const uint32_t ticks_window = ticks_raw - ticks_raw_prev;
    const uint32_t seq_delta = seq - seq_prev;
    const uint32_t elapsed_us = pps_time_us - pps_time_us_prev;
    const uint64_t expected_us =
        (uint64_t)seq_delta * 1000000ULL;
    const uint64_t timing_error_us =
        (elapsed_us >= expected_us)
            ? (uint64_t)elapsed_us - expected_us
            : expected_us - (uint64_t)elapsed_us;
    const bool interval_valid =
        seq_delta > 0 &&
        timing_error_us <= PPS_TIME_TOLERANCE_US;
    uint32_t ticks = 0;
    int64_t diff = 0;
    bool diff_outlier = false;

    // No acumulamos offset ni hacemos predict/update antes de que se sincronicen con la primer UTC válida.
    if (interval_valid && seq_delta == 1 && !venia_de_holdover) {
        const int64_t diff_candidato = (int64_t)ticks_window - 32768LL;
        diff_outlier = (diff_candidato > MAX_DIFF_TICKS) ||
                       (diff_candidato < -MAX_DIFF_TICKS);
    }
    const bool pps_apto_para_inicio =
        pending == 1 && seq_delta == 1 && interval_valid && !venia_de_holdover;
    const bool sincronizado_antes = rtc_sincronizado_utc;
    sincronizarRtcConUtcEnPps(seq, pps_time_us, pps_apto_para_inicio);

    if (!sincronizado_antes) {
        // Primer PPS con UTC validada: es el ORIGEN, NO una medicion.
        if (rtc_sincronizado_utc) {
            offset_ticks = 0;
            kalman.reset();
            en_holdover = false;
            holdover_steps = 0;
        }
        Kalman_State estado_inicial = kalman.getState();
        char rtc_gnss_fields[1024];
        buildRtcGnssJsonFields(rtc_gnss_fields, sizeof(rtc_gnss_fields),
                               rtc_sincronizado_utc && pps_apto_para_inicio,
                               seq, ticks_raw,
                               rtc_sincronizado_utc ? &estado_inicial : nullptr);
        char json[1800];
        buildPpsPreUtcLogJson(
            json, sizeof(json), DEVICE_ID, (int)aging_aplicado,
            now_ms, seq, pending, pps_period_us,
            ticks_raw, ticks_window, seq_delta, elapsed_us,
            interval_valid, diff_outlier, rtc_sincronizado_utc,
            rtc_gnss_fields);
        telemetryQueue.enqueue(json);
        ticks_raw_prev = ticks_raw;
        seq_prev = seq;
        pps_time_us_prev = pps_time_us;
        return;
    }

    
    //Si veníamos de holdover, esta primera muestra se usa
    //solamente para reenganchar la referencia. No se usa para actualizar Kalman.

    if (interval_valid && !venia_de_holdover) {
        ticks = ticks_window / seq_delta;
        const int64_t expected_ticks =
            (int64_t)32768 * (int64_t)seq_delta;
        // Convención:
        // diff = ticks_RTC - ticks_esperados_GNSS
        diff = (int64_t)ticks_window - expected_ticks;
        diff_outlier =
            (diff > MAX_DIFF_TICKS) ||
            (diff < -MAX_DIFF_TICKS);
        if (!diff_outlier) {
            offset_ticks += diff;
        }
    }
    bool kalman_actualizado = false;
    bool kalman_prediccion = false;
    Kalman_State kalman_state;
    if (
        interval_valid &&
        seq_delta == 1 &&
        !diff_outlier &&
        !venia_de_holdover
    ) {
        // Etapa de predicción
        kalman.predict();
        kalman_prediccion = true;
        // Etapa de actualización con medición escalar:
        // z = phi
        kalman.updatePhi((double)offset_ticks);
        kalman_state = kalman.getState();
        kalman_actualizado = true;
        if (estadistica_inicio_pps_seq == 0) {
            estadistica_inicio_pps_seq = seq;
        }
    } else {
        kalman_state = kalman.getState();
        kalman_actualizado = false;
        kalman_prediccion = false;
    }
    const uint32_t elapsed_seconds_rounded =
        (elapsed_us + 500000UL) / 1000000UL;
    const uint32_t missed_pps =
        (elapsed_seconds_rounded > seq_delta)
            ? elapsed_seconds_rounded - seq_delta
            : 0;

    // Reconstruir solo los segundos respaldados por micros() Y el RTC 32 kHz.
    threeClocks.observePpsInterval(
        venia_de_holdover, elapsed_us, seq_delta, pending, ticks_window);
    const bool offset_medido_disponible =
        interval_valid &&
        !diff_outlier &&
        !venia_de_holdover;
    const char* modo = "PPS_SIN_UPDATE_KALMAN";
    if (venia_de_holdover) {
        modo = "PPS_REENGANCHE";
    } else if (kalman_actualizado) {
        modo = "PPS_KALMAN_Z_PHI";
    }
    char rtc_gnss_fields[1024];
    const bool pps_apto_para_relojes =
        pending == 1 && seq_delta == 1 && kalman_actualizado && !venia_de_holdover;
    // RTC y epoca se inicializaron previamente, en el PPS de origen.
    buildRtcGnssJsonFields(rtc_gnss_fields, sizeof(rtc_gnss_fields),
                           pps_apto_para_relojes, seq, ticks_raw,
                           &kalman_state);

    char json[1800];
    buildPpsLogJson(
        json,
        sizeof(json),
        DEVICE_ID,
        (int)aging_aplicado,
        now_ms,
        seq,
        pending,
        pps_period_us,
        modo,
        venia_de_holdover ? holdover_steps_previos : 0,
        offset_medido_disponible,
        kalman_prediccion,
        kalman_actualizado,
        ticks,
        ticks_window,
        ticks_raw,
        seq_delta,
        elapsed_us,
        interval_valid,
        diff_outlier,
        missed_pps,
        diff,
        offset_ticks,
        TICK_US,
        kalman_state,
        rtc_gnss_fields
    );
    telemetryQueue.enqueue(json);
    ticks_raw_prev = ticks_raw;
    seq_prev = seq;
    pps_time_us_prev = pps_time_us;
    if (venia_de_holdover) {
        en_holdover = false;
        holdover_steps = 0;
    }
}