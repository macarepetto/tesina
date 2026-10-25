#include <Arduino.h>
#include "driver/pcnt.h"
#include "Kalman_Filter.h"
#include "GPS_Module.h"
#include "RTC_Module.h"
#include "WiFi_Module.h"
#include "Telemetry_Queue_Module.h"
#include "Json_Log_Builder.h"

// ==================== PINES ====================

#define PIN_32K_RTC 18
#define PIN_PPS_GPS 27

// ==================== GNSS UART / NMEA ====================
#define PIN_GPS_RX 16
#define PIN_GPS_TX 17
#define GPS_BAUD 9600

GPS_Module gps(Serial2, PIN_GPS_RX, PIN_GPS_TX, PIN_PPS_GPS, GPS_BAUD);
RTC_Module rtc;

// ==================== PCNT ====================
#define PCNT_UNIT_USED    PCNT_UNIT_0
#define PCNT_CHANNEL_USED PCNT_CHANNEL_0
#define PCNT_HIGH_LIMIT 30000
#define PCNT_LOW_LIMIT -30000
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

// ==================== WIFI ====================
const char* WIFI_SSID = "PATAN";
const char* WIFI_PASS = "autoslocos";
const char* SERVER_IP = "192.168.0.119";
const uint16_t SERVER_PORT = 8080;
const char* DEVICE_ID = "esp32-prototipo-1";

//WiFiClient client;

WiFi_Module wifiModule(
    WIFI_SSID,
    WIFI_PASS,
    SERVER_IP,
    SERVER_PORT,
    DEVICE_ID
);

// ==================== KALMAN ====================
Kalman_Filter kalman;

// ==================== VARIABLES COMPARTIDAS ISR / LOOP ====================
portMUX_TYPE pcnt_mux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t pcnt_base = 0;
volatile uint32_t ticks_raw_isr = 0;
volatile uint32_t pps_seq = 0;
volatile uint32_t pps_pending = 0;
volatile uint32_t pps_time_us_isr = 0;
volatile uint32_t pps_period_us_isr = 0;

int8_t aging_aplicado = 0;

// ==================== FECHA/HORA GNSS + RTC ====================

// Para el log: consideramos GNSS válido sólo si la última RMC válida es reciente.
static constexpr uint32_t GNSS_VALID_LOG_MAX_AGE_MS = 3000;

// Ejecutar tareas secundarias con un ritmo acotado.
// La medición PPS/PCNT se procesa primero; GNSS queda en background.
// La comunicación WiFi se procesa en CORE 0 con Telemetry_Queue_Module.
static constexpr uint32_t BACKGROUND_TASKS_PERIOD_MS = 20;

// Cola + tarea de salida WiFi (CORE 0). El loop sólo encola cada JSON.
Telemetry_Queue_Module telemetryQueue(wifiModule);

// ==================== TRES RELOJES RELATIVOS ====================
// Este comparador NO altera la hora interna del DS3231 ni el filtro.
// En el primer PPS con RMC reciente fijamos una época candidata común.
// GNSS: un segundo entero por PPS. RTC: tiempo transcurrido según 32 kHz.
// Kalman: tiempo RTC menos la deriva relativa estimada (phi - phi_inicio).
// La asociación entre la época RMC y el pulso físico AÚN NO ESTÁ VALIDADA.
struct TresRelojesBase {
    bool disponible = false;
    uint32_t seq_inicio = 0;
    uint32_t ticks_inicio = 0;
    uint32_t epoch_local_inicio = 0;
    double phi_inicio = 0.0;
};
static TresRelojesBase tresRelojesBase;

static int64_t redondearMicrosegundos(double value) {
    return (int64_t)(value >= 0.0 ? value + 0.5 : value - 0.5);
}

static void formatoHoraLocalUs(int64_t timestamp_us, char* out, size_t len) {
    const uint32_t epoch_s = (uint32_t)(timestamp_us / 1000000LL);
    const unsigned long fraccion_us =
        (unsigned long)(timestamp_us % 1000000LL);
    const DateTime dt(epoch_s);
    snprintf(out, len, "%04u-%02u-%02uT%02u:%02u:%02u.%06lu",
        (unsigned)dt.year(), (unsigned)dt.month(), (unsigned)dt.day(),
        (unsigned)dt.hour(), (unsigned)dt.minute(), (unsigned)dt.second(),
        fraccion_us);
}

static void anexarTresRelojes(
    char* out, size_t len, bool pps_apto,
    uint32_t seq, uint32_t ticks_raw,
    const Kalman_State* kalman_state,
    bool rmc_reciente, const DateTime& gnss_local)
{
    if (len == 0) return;
    const size_t used = strlen(out);
    if (used >= len) return;

    // La época solo se establece en un PPS sin backlog y con Kalman actualizado.
    // El dato RMC sirve para etiquetar las tres series; no valida fase absoluta.
    if (!tresRelojesBase.disponible && pps_apto &&
        kalman_state != nullptr && rmc_reciente) {
        tresRelojesBase.disponible = true;
        tresRelojesBase.seq_inicio = seq;
        tresRelojesBase.ticks_inicio = ticks_raw;
        tresRelojesBase.epoch_local_inicio = gnss_local.unixtime();
        tresRelojesBase.phi_inicio = kalman_state->phi;
    }

    const bool utilizable = tresRelojesBase.disponible && pps_apto &&
        kalman_state != nullptr && rmc_reciente &&
        seq >= tresRelojesBase.seq_inicio &&
        (seq - tresRelojesBase.seq_inicio) < 64800UL;

    if (!utilizable) {
        snprintf(out + used, len - used,
            ",\"reloj_gnss_pps\":null,"
            "\"reloj_rtc_32k\":null,"
            "\"reloj_kalman\":null,"
            "\"desfase_rtc_us\":null,"
            "\"desfase_kalman_us\":null,"
            "\"relojes_estado\":\"SIN_MUESTRA_COMPARABLE\"");
        return;
    }

    const uint32_t segundos = seq - tresRelojesBase.seq_inicio;
    const uint32_t ticks_32k = ticks_raw - tresRelojesBase.ticks_inicio;
    const double transcurrido_rtc_us = (double)ticks_32k * TICK_US;
    const double transcurrido_gnss_us = (double)segundos * 1000000.0;
    // Desfase respecto del PPS inicial, no fase absoluta del calendario RTC.
    const double desfase_rtc_us = transcurrido_rtc_us - transcurrido_gnss_us;
    const double desfase_kalman_us = desfase_rtc_us -
        (kalman_state->phi - tresRelojesBase.phi_inicio) * TICK_US;

    const int64_t referencia_us =
        ((int64_t)tresRelojesBase.epoch_local_inicio + segundos) * 1000000LL;
    char hora_gnss[40], hora_rtc[40], hora_kalman[40];
    formatoHoraLocalUs(referencia_us, hora_gnss, sizeof(hora_gnss));
    formatoHoraLocalUs(referencia_us + redondearMicrosegundos(desfase_rtc_us),
                       hora_rtc, sizeof(hora_rtc));
    formatoHoraLocalUs(referencia_us + redondearMicrosegundos(desfase_kalman_us),
                       hora_kalman, sizeof(hora_kalman));

    snprintf(out + used, len - used,
        ",\"reloj_gnss_pps\":\"%s\","
        "\"reloj_rtc_32k\":\"%s\","
        "\"reloj_kalman\":\"%s\","
        "\"desfase_rtc_us\":%.3f,"
        "\"desfase_kalman_us\":%.3f,"
        "\"relojes_estado\":\"RELATIVO_EPOCA_RMC_NO_VALIDADA\"",
        hora_gnss, hora_rtc, hora_kalman,
        desfase_rtc_us, desfase_kalman_us);
}

// El RTC se sincroniza una sola vez: con la primera RMC válida recibida.
static bool rtc_sincronizado = false;

void sincronizarRtcConPrimerGnssValido() {
    if (rtc_sincronizado) {
        return;
    }

    if (!gps.hasValidRmcRecent(GNSS_VALID_LOG_MAX_AGE_MS)) {
        return;
    }

    DateTime gps_local(2000, 1, 1, 0, 0, 0);
    if (!gps.getLastValidLocalDateTime(gps_local)) {
        return;
    }

    if (rtc.setFromDateTime(gps_local)) {
        rtc_sincronizado = true;

        char iso[32];
        RTC_Module::dateTimeToIsoLocal(gps_local, iso, sizeof(iso));
        Serial.print("RTC sincronizado una sola vez con GNSS local: ");
        Serial.println(iso);
    } else {
        Serial.println("ERROR: no se pudo sincronizar el RTC con GNSS");
    }
}

void buildRtcGnssJsonFields(char* out, size_t len,
                            bool pps_apto = false,
                            uint32_t seq = 0,
                            uint32_t ticks_raw = 0,
                            const Kalman_State* kalman_state = nullptr) {
    char rtc_json[40];
    char gnss_json[40];

    snprintf(rtc_json, sizeof(rtc_json), "null");
    snprintf(gnss_json, sizeof(gnss_json), "null");

    DateTime rtc_dt(2000, 1, 1, 0, 0, 0);
    const bool rtc_ok = rtc.readDateTime(rtc_dt);
    if (rtc_ok) {
        char rtc_iso[32];
        RTC_Module::dateTimeToIsoLocal(rtc_dt, rtc_iso, sizeof(rtc_iso));
        snprintf(rtc_json, sizeof(rtc_json), "\"%s\"", rtc_iso);
    }

    DateTime gnss_local(2000, 1, 1, 0, 0, 0);
    const bool gnss_ok =
        gps.hasValidRmcRecent(GNSS_VALID_LOG_MAX_AGE_MS) &&
        gps.getLastValidLocalDateTime(gnss_local);

    if (gnss_ok) {
        char gnss_iso[32];
        RTC_Module::dateTimeToIsoLocal(gnss_local, gnss_iso, sizeof(gnss_iso));
        snprintf(gnss_json, sizeof(gnss_json), "\"%s\"", gnss_iso);
    }

    snprintf(
        out,
        len,
        ",\"gnss_valido\":%s,\"rtc_hora\":%s,\"gnss_hora\":%s",
        gnss_ok ? "true" : "false",
        rtc_json,
        gnss_json
    );

    // La comprobación de RMC se hace sin nuevas operaciones UART ni I2C.
    anexarTresRelojes(out, len, pps_apto, seq, ticks_raw, kalman_state,
        gnss_ok && gps.hasValidRmcRecent(1500), gnss_local);
}


void runBackgroundTasks() {
    static uint32_t last_background_ms = 0;

    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - last_background_ms) < BACKGROUND_TASKS_PERIOD_MS) {
        return;
    }

    last_background_ms = now_ms;

    gps.update();
    sincronizarRtcConPrimerGnssValido();
}

// ==================== INTERRUPCIONES ====================

void IRAM_ATTR onPCNTLimit(void* arg) {
    uint32_t status = 0;
    pcnt_get_event_status(PCNT_UNIT_USED, &status);
    if (status & PCNT_EVT_H_LIM) {
        portENTER_CRITICAL_ISR(&pcnt_mux);
        pcnt_base += PCNT_HIGH_LIMIT;
        portEXIT_CRITICAL_ISR(&pcnt_mux);
    }
}

void IRAM_ATTR onPPS() {
    const uint32_t now_us = micros();
    int16_t hw_count = 0;
    portENTER_CRITICAL_ISR(&pcnt_mux);
    pcnt_get_counter_value(PCNT_UNIT_USED, &hw_count);
    if (pps_time_us_isr != 0) {
        pps_period_us_isr = now_us - pps_time_us_isr;
    } else {
        pps_period_us_isr = 0;
    }
    pps_time_us_isr = now_us;
    ticks_raw_isr = pcnt_base + (uint32_t)hw_count;
    pps_seq++;
    pps_pending++;
    portEXIT_CRITICAL_ISR(&pcnt_mux);
}

// ==================== CONFIGURACIÓN ====================

void setupPCNT() {
    pinMode(PIN_32K_RTC, INPUT_PULLUP);
    pcnt_config_t config = {};
    config.pulse_gpio_num = PIN_32K_RTC;
    config.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    config.pos_mode = PCNT_COUNT_INC;
    config.neg_mode = PCNT_COUNT_DIS;
    config.lctrl_mode = PCNT_MODE_KEEP;
    config.hctrl_mode = PCNT_MODE_KEEP;
    config.counter_h_lim = PCNT_HIGH_LIMIT;
    config.counter_l_lim = PCNT_LOW_LIMIT;
    config.unit = PCNT_UNIT_USED;
    config.channel = PCNT_CHANNEL_USED;
    if (pcnt_unit_config(&config) != ESP_OK) {
        Serial.println("ERROR: no se pudo configurar PCNT");
        while (true) {
            delay(1000);
        }
    }
    pcnt_filter_disable(PCNT_UNIT_USED);
    pcnt_counter_pause(PCNT_UNIT_USED);
    pcnt_counter_clear(PCNT_UNIT_USED);
    pcnt_base = 0;
    pcnt_event_enable(PCNT_UNIT_USED, PCNT_EVT_H_LIM);
    esp_err_t result = pcnt_isr_service_install(0);
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        Serial.println("ERROR: no se pudo instalar la ISR de PCNT");
        while (true) {
            delay(1000);
        }
    }
    if (pcnt_isr_handler_add(
            PCNT_UNIT_USED,
            onPCNTLimit,
            nullptr
        ) != ESP_OK) {
        Serial.println("ERROR: no se pudo registrar la ISR de PCNT");
        while (true) {
            delay(1000);
        }
    }
    pcnt_counter_resume(PCNT_UNIT_USED);
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

    // No llamamos gps.begin() porque ese begin() también engancha una ISR al PPS.
    // El PPS de máxima prioridad sigue siendo únicamente onPPS().
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
    setupPCNT();
    pinMode(PIN_PPS_GPS, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_PPS_GPS), onPPS, RISING);
    if (!telemetryQueue.begin()) {
        Serial.println("ERROR: no se pudo crear la cola/tarea de telemetria");
        while (true) {
            delay(1000);
        }
    }
    Serial.println("PCNT iniciado. Telemetria en CORE 0. Esperando PPS...");
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
    uint32_t ticks_raw = 0;
    uint32_t seq = 0;
    uint32_t pending = 0;
    uint32_t pps_time_us = 0;
    uint32_t pps_period_us = 0;
    bool hay_muestra = false;
    portENTER_CRITICAL(&pcnt_mux);
    if (pps_pending > 0) {
        ticks_raw = ticks_raw_isr;
        seq = pps_seq;
        pending = pps_pending;
        pps_time_us = pps_time_us_isr;
        pps_period_us = pps_period_us_isr;
        pps_pending = 0;
        hay_muestra = true;
    }
    portEXIT_CRITICAL(&pcnt_mux);
    // ============================================================
    // CASO 1: NO LLEGÓ PPS NUEVO
    // ============================================================
    if (!hay_muestra) {
        runBackgroundTasks();

        if (!hay_anterior) {
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
            // En holdover NO hay medición nueva.
            // Solo se aplica la etapa de predicción:
            //
            //     phi(k+1) = phi(k) + f(k)
            //     f(k+1)   = f(k)
            kalman.predict();
            Kalman_State kalman_state = kalman.getState();
            char rtc_gnss_fields[480];
            buildRtcGnssJsonFields(rtc_gnss_fields, sizeof(rtc_gnss_fields));

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
        char rtc_gnss_fields[480];
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
    /*
        Si veníamos de holdover, esta primera muestra se usa
        solamente para reenganchar la referencia.
        No se usa para actualizar offset ni Kalman, porque el intervalo
        entre el último PPS antes de la pérdida y este nuevo PPS no
        representa una ventana normal de 1 segundo.
    */
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
    char rtc_gnss_fields[480];
    const bool pps_apto_para_relojes =
        pending == 1 && seq_delta == 1 && kalman_actualizado && !venia_de_holdover;
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