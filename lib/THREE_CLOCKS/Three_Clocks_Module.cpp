#include "Three_Clocks_Module.h"

#include <stdio.h>
#include <string.h>

namespace {
constexpr double TICK_US = 1000000.0 / 32768.0;
constexpr uint32_t MAX_SEGMENT_SECONDS = 64800UL;  // 18 horas
constexpr uint32_t PPS_INTERVAL_TOLERANCE_US = 200000UL;
}

void Three_Clocks_Module::establishBase(uint32_t seq, uint32_t ticks_raw,
                                        uint32_t epoch, double phi) {
    base_.available = true;
    base_.seq_start = seq;
    base_.ticks_start = ticks_raw;
    base_.epoch_start = epoch;
    base_.start_ms = millis();
    base_.phi_start = phi;
    recovered_seconds_ = 0;
    reference_trusted_ = true;
    clean_reanchor_samples_ = 0;
    ++segment_;
}

bool Three_Clocks_Module::elapsedSeconds(uint32_t seq,
                                         uint32_t& seconds) const {
    if (!base_.available || seq < base_.seq_start) return false;
    const uint64_t total = (uint64_t)(seq - base_.seq_start) +
                           recovered_seconds_;
    if (total >= MAX_SEGMENT_SECONDS) return false;
    seconds = (uint32_t)total;
    return true;
}

void Three_Clocks_Module::observePpsInterval(
    bool coming_from_holdover, uint32_t elapsed_us,
    uint32_t seq_delta, uint32_t pending, uint32_t ticks_window) {
    if (!base_.available || !coming_from_holdover) return;

    // El micros() de ESP32 desborda cada ~71 min. Si el intervalo fue mayor,
    // no tenemos prueba suficiente para reconstruirlo: forzar reanclaje.
    const uint32_t rounded = (uint32_t)(((uint64_t)elapsed_us + 500000ULL) /
                                        1000000ULL);
    const uint64_t reference_us = (uint64_t)rounded * 1000000ULL;
    const uint64_t error_us = elapsed_us >= reference_us
        ? (uint64_t)elapsed_us - reference_us
        : reference_us - (uint64_t)elapsed_us;
    const int64_t ticks_expected = (int64_t)rounded * 32768LL;
    const int64_t ticks_error = (int64_t)ticks_window - ticks_expected;
    const int64_t tick_tolerance = 10LL * rounded;

    if (pending == 1 && seq_delta == 1 && rounded >= 2 &&
        error_us <= PPS_INTERVAL_TOLERANCE_US &&
        ticks_error >= -tick_tolerance && ticks_error <= tick_tolerance &&
        recovered_seconds_ <= MAX_SEGMENT_SECONDS - rounded) {
        // Hay (rounded - seq_delta) segundos reales sin ISR PPS.
        recovered_seconds_ += rounded - seq_delta;
    } else if (rounded > 1 || pending != 1 || seq_delta != 1) {
        reference_trusted_ = false;
        clean_reanchor_samples_ = 0;
    }
}

bool Three_Clocks_Module::differenceFromRmc(
    uint32_t seq, bool rmc_recent, const DateTime& rmc_local,
    int64_t& diff_s) const {
    uint32_t seconds = 0;
    if (!rmc_recent || !elapsedSeconds(seq, seconds)) return false;
    diff_s = (int64_t)rmc_local.unixtime() -
             ((int64_t)base_.epoch_start + seconds);
    return true;
}

void Three_Clocks_Module::trackReanchor(
    uint32_t seq, uint32_t ticks_raw,
    const DateTime& gnss_local, const Kalman_State& state) {
    const uint32_t epoch = gnss_local.unixtime();
    if (clean_reanchor_samples_ > 0 &&
        seq == reanchor_prev_seq_ + 1 &&
        epoch == reanchor_prev_epoch_ + 1) {
        ++clean_reanchor_samples_;
    } else {
        clean_reanchor_samples_ = 1;
    }
    reanchor_prev_seq_ = seq;
    reanchor_prev_epoch_ = epoch;

    // Tres PPS regulares con tres etiquetas UTC consecutivas y UBX valido.
    // Esto RECUPERA EL COMPARADOR; NO prueba la fase fisica GNSS-PPS.
    if (clean_reanchor_samples_ >= 3) {
        establishBase(seq, ticks_raw, epoch, state.phi);
        ++reanchors_;
    }
}

int64_t Three_Clocks_Module::roundMicroseconds(double value) {
    return (int64_t)(value >= 0.0 ? value + 0.5 : value - 0.5);
}

void Three_Clocks_Module::formatLocalTimeUs(
    int64_t timestamp_us, char* out, size_t len) {
    const uint32_t epoch_s = (uint32_t)(timestamp_us / 1000000LL);
    const unsigned long fraction_us =
        (unsigned long)(timestamp_us % 1000000LL);
    const DateTime dt(epoch_s);
    snprintf(out, len, "%04u-%02u-%02uT%02u:%02u:%02u.%06lu",
        (unsigned)dt.year(), (unsigned)dt.month(), (unsigned)dt.day(),
        (unsigned)dt.hour(), (unsigned)dt.minute(), (unsigned)dt.second(),
        fraction_us);
}

void Three_Clocks_Module::appendHoldoverJson(
    char* out, size_t len, uint32_t live_ticks_raw,
    const Kalman_State* state) {
    if (len == 0) return;
    const size_t used = strlen(out);
    if (used >= len) return;

    // La referencia inicial tiene que haber quedado establecida con UTC.
    // El limite de 18 h evita confundir un desborde del contador de ticks
    // con un intervalo corto; nunca inventamos una fase UTC nueva.
    const bool usable = base_.available && reference_trusted_ && state &&
        (uint32_t)(millis() - base_.start_ms) <
            MAX_SEGMENT_SECONDS * 1000UL;
    if (!usable) {
        snprintf(out + used, len - used,
            ",\"reloj_gnss_pps\":null,"
            "\"reloj_rtc_32k\":null,"
            "\"reloj_kalman\":null,"
            "\"desfase_rtc_us\":null,"
            "\"desfase_kalman_us\":null,"
            "\"relojes_estado\":\"%s\","
            "\"relojes_segmento\":%lu,"
            "\"relojes_segundos_recuperados\":%lu,"
            "\"relojes_reanclajes\":%lu",
            reference_trusted_ ? "SIN_MUESTRA_COMPARABLE" :
                                 "REFERENCIA_NO_CONFIABLE",
            (unsigned long)segment_, (unsigned long)recovered_seconds_,
            (unsigned long)reanchors_);
        return;
    }

    const uint32_t elapsed_ticks = live_ticks_raw - base_.ticks_start;
    const double rtc_elapsed_us = (double)elapsed_ticks * TICK_US;
    const double kalman_elapsed_us = rtc_elapsed_us -
        (state->phi - base_.phi_start) * TICK_US;
    const int64_t epoch_us = (int64_t)base_.epoch_start * 1000000LL;
    char rtc_time[40], kalman_time[40];
    formatLocalTimeUs(epoch_us + roundMicroseconds(rtc_elapsed_us),
                      rtc_time, sizeof(rtc_time));
    formatLocalTimeUs(epoch_us + roundMicroseconds(kalman_elapsed_us),
                      kalman_time, sizeof(kalman_time));
    snprintf(out + used, len - used,
        ",\"reloj_gnss_pps\":null,"
        "\"reloj_rtc_32k\":\"%s\","
        "\"reloj_kalman\":\"%s\","
        "\"desfase_rtc_us\":null,"
        "\"desfase_kalman_us\":null,"
        "\"relojes_estado\":\"HOLDOVER_PREDICHO\","
        "\"relojes_segmento\":%lu,"
        "\"relojes_segundos_recuperados\":%lu,"
        "\"relojes_reanclajes\":%lu",
        rtc_time, kalman_time,
        (unsigned long)segment_, (unsigned long)recovered_seconds_,
        (unsigned long)reanchors_);
}

void Three_Clocks_Module::appendJson(
    char* out, size_t len, bool pps_usable,
    uint32_t seq, uint32_t ticks_raw,
    const Kalman_State* state, bool rmc_recent,
    const DateTime& gnss_local, bool rtc_synced,
    uint32_t rtc_sync_pps_seq, bool utc_reference_ready) {
    if (len == 0) return;
    const size_t used = strlen(out);
    if (used >= len) return;

    if (!base_.available && pps_usable && rtc_synced &&
        rtc_sync_pps_seq == seq && state && rmc_recent) {
        establishBase(seq, ticks_raw, gnss_local.unixtime(), state->phi);
    }

    // Hay dos validaciones diferentes:
    // 1) la base historica debe seguir siendo coherente con RMC;
    // 2) si se perdio, podemos REANCLAR sin depender de esa base vieja.
    // Un contador seq contaminado por falsos PPS puede superar las 18 h
    // ficticias aun cuando los nuevos PPS sean completamente regulares.
    const bool sample_ready = pps_usable && state && rmc_recent;
    uint32_t seconds = 0;
    const bool old_base_usable = elapsedSeconds(seq, seconds);

    // Una consulta de diagnosticos sin muestra PPS (seq=0, state=nullptr)
    // NO debe invalidar el segmento vigente.
    if (base_.available && reference_trusted_ && sample_ready) {
        int64_t rmc_difference = 0;
        const bool rmc_mismatch = sample_ready && old_base_usable &&
            differenceFromRmc(seq, rmc_recent, gnss_local,
                              rmc_difference) && rmc_difference != 0;
        if (!old_base_usable || rmc_mismatch) {
            // Se invalida UNA sola vez la referencia anterior. No se vuelve
            // a borrar la cuenta de muestras limpias en cada segundo por
            // seguir discrepando con la base que YA dejamos de confiar.
            reference_trusted_ = false;
            clean_reanchor_samples_ = 0;
        }
    }

    bool reanchored_now = false;
    if (base_.available && !reference_trusted_) {
        if (sample_ready && utc_reference_ready) {
            // Tres muestras con secuencias y etiquetas UTC consecutivas.
            // NO exigimos que coincidan con la epoca de la base danada.
            const uint32_t previous_reanchors = reanchors_;
            trackReanchor(seq, ticks_raw, gnss_local, *state);
            reanchored_now = reanchors_ != previous_reanchors;
        } else {
            // Si falla la validez del PPS/UTC, la racha se interrumpe.
            clean_reanchor_samples_ = 0;
        }
    }

    // Si hubo reanclaje, elapsedSeconds vuelve a trabajar con el nuevo
    // origen y ya no arrastra los miles de flancos espurios anteriores.
    const bool usable = sample_ready && reference_trusted_ &&
                        elapsedSeconds(seq, seconds);

    const size_t at = strlen(out);
    if (!usable) {
        snprintf(out + at, len - at,
            ",\"reloj_gnss_pps\":null,"
            "\"reloj_rtc_32k\":null,"
            "\"reloj_kalman\":null,"
            "\"desfase_rtc_us\":null,"
            "\"desfase_kalman_us\":null,"
            "\"relojes_estado\":\"%s\","
            "\"relojes_segmento\":%lu,"
            "\"relojes_segundos_recuperados\":%lu,"
            "\"relojes_reanclajes\":%lu",
            reference_trusted_ ? "SIN_MUESTRA_COMPARABLE" :
                                 "REFERENCIA_NO_CONFIABLE",
            (unsigned long)segment_, (unsigned long)recovered_seconds_,
            (unsigned long)reanchors_);
        return;
    }

    const uint32_t ticks_32k = ticks_raw - base_.ticks_start;
    const double rtc_elapsed_us = (double)ticks_32k * TICK_US;
    const double gnss_elapsed_us = (double)seconds * 1000000.0;
    const double rtc_drift_us = rtc_elapsed_us - gnss_elapsed_us;
    const double kalman_drift_us = rtc_drift_us -
                                   (state->phi - base_.phi_start) * TICK_US;
    const int64_t reference_us =
        ((int64_t)base_.epoch_start + seconds) * 1000000LL;

    char gnss_time[40], rtc_time[40], kalman_time[40];
    formatLocalTimeUs(reference_us, gnss_time, sizeof(gnss_time));
    formatLocalTimeUs(reference_us + roundMicroseconds(rtc_drift_us),
                      rtc_time, sizeof(rtc_time));
    formatLocalTimeUs(reference_us + roundMicroseconds(kalman_drift_us),
                      kalman_time, sizeof(kalman_time));
    snprintf(out + at, len - at,
        ",\"reloj_gnss_pps\":\"%s\","
        "\"reloj_rtc_32k\":\"%s\","
        "\"reloj_kalman\":\"%s\","
        "\"desfase_rtc_us\":%.3f,"
        "\"desfase_kalman_us\":%.3f,"
        "\"relojes_estado\":\"%s\","
        "\"relojes_segmento\":%lu,"
        "\"relojes_segundos_recuperados\":%lu,"
        "\"relojes_reanclajes\":%lu",
        gnss_time, rtc_time, kalman_time, rtc_drift_us, kalman_drift_us,
        reanchored_now ? "RELATIVO_REANCLADO_RMC_PPS" :
                         "RELATIVO_UTC_COHERENTE",
        (unsigned long)segment_, (unsigned long)recovered_seconds_,
        (unsigned long)reanchors_);
}
