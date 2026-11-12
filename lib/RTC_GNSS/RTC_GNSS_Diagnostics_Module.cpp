#include "RTC_GNSS_Diagnostics_Module.h"

#include <stdio.h>
#include <string.h>
#include "PPS_Capture.h"

namespace {
constexpr uint32_t GNSS_VALID_LOG_MAX_AGE_MS = 3000;
}

RTC_GNSS_Diagnostics_Module::RTC_GNSS_Diagnostics_Module(
    GPS_Module& gps, RTC_Module& rtc, Three_Clocks_Module& clocks)
    : gps_(gps), rtc_(rtc), clocks_(clocks) {}

void RTC_GNSS_Diagnostics_Module::observeRmcAfterPps() {
    const RmcDiagnostics d = gps_.getRmcDiagnostics();
    if (d.revision == 0 || d.revision == last_rmc_revision_) return;
    last_rmc_revision_ = d.revision;
    uint32_t seq_snapshot = 0;
    uint32_t pps_us_snapshot = 0;
    PPS_Capture::snapshotLastPps(seq_snapshot, pps_us_snapshot);
    last_rmc_pps_seq_ = seq_snapshot;
    const int32_t lag_us = (int32_t)(d.procesado_us - pps_us_snapshot);
    last_rmc_after_pps_us_ =
        (seq_snapshot != 0 && lag_us >= 0 && lag_us <= 1000000)
        ? lag_us : -1;
}

void RTC_GNSS_Diagnostics_Module::appendRmc(
    char* out, size_t len, uint32_t seq,
    bool rmc_recent, const DateTime& rmc_local) const {
    const size_t used = strlen(out);
    if (used >= len) return;
    const RmcDiagnostics d = gps_.getRmcDiagnostics();
    char diff_json[24] = "null";
    int64_t diff_s = 0;
    if (clocks_.differenceFromRmc(seq, rmc_recent, rmc_local, diff_s)) {
        snprintf(diff_json, sizeof(diff_json), "%lld", (long long)diff_s);
    }
    char lag_json[24] = "null";
    if (last_rmc_after_pps_us_ >= 0 && d.revision != 0) {
        snprintf(lag_json, sizeof(lag_json), "%ld",
                 (long)last_rmc_after_pps_us_);
    }
    snprintf(out + used, len - used,
        ",\"rmc_checksum_fallos\":%lu,"
        "\"rmc_rechazadas\":%lu,"
        "\"rmc_saltos\":%lu,"
        "\"rmc_delta_s\":%ld,"
        "\"rmc_error_continuidad_s\":%ld,"
        "\"rmc_intervalo_ms\":%lu,"
        "\"rmc_edad_ms\":%lu,"
        "\"rmc_vs_pps_s\":%s,"
        "\"rmc_post_pps_us\":%s,"
        "\"rmc_pps_seq\":%lu",
        (unsigned long)d.checksum_fallidos,
        (unsigned long)d.rechazadas,
        (unsigned long)d.saltos,
        (long)d.ultimo_delta_s,
        (long)d.ultimo_error_continuidad_s,
        (unsigned long)d.intervalo_ms,
        (unsigned long)(d.revision ? gps_.getLastValidRmcAgeMs() : 0UL),
        diff_json, lag_json, (unsigned long)last_rmc_pps_seq_);
}

void RTC_GNSS_Diagnostics_Module::appendUbx(char* out, size_t len) const {
    const size_t used = strlen(out);
    if (used >= len) return;
    const UbxUtcDiagnostics u = gps_.getUbxUtcDiagnostics();
    char time_json[40] = "null";
    if (u.recibido && u.fecha_razonable) {
        const DateTime utc(u.year, u.month, u.day,
                           u.hour, u.minute, u.second);
        const uint32_t epoch = utc.unixtime() - 3UL * 3600UL;
        char iso[32];
        RTC_Module::dateTimeToIsoLocal(DateTime(epoch), iso, sizeof(iso));
        snprintf(time_json, sizeof(time_json), "\"%s\"", iso);
    }
    const char* valid_json = !u.recibido ? "null" :
        ((u.flags & 0x07) == 0x07 ? "true" : "false");
    const uint32_t age = u.recibido ? (millis() - u.recibido_ms) : 0UL;
    snprintf(out + used, len - used,
        ",\"ubx_polls\":%lu,"
        "\"ubx_respuestas\":%lu,"
        "\"ubx_checksum_fallos\":%lu,"
        "\"ubx_valid_utc\":%s,"
        "\"ubx_flags\":%u,"
        "\"ubx_utc_hora\":%s,"
        "\"ubx_tacc_ns\":%lu,"
        "\"ubx_nano_ns\":%ld,"
        "\"ubx_itow_ms\":%lu,"
        "\"ubx_edad_ms\":%lu",
        (unsigned long)u.consultas,
        (unsigned long)u.respuestas,
        (unsigned long)u.checksum_fallos,
        valid_json, (unsigned)u.flags, time_json,
        (unsigned long)u.tAcc_ns, (long)u.nano_ns,
        (unsigned long)u.iTOW_ms, (unsigned long)age);
}

void RTC_GNSS_Diagnostics_Module::appendInit(
    char* out, size_t len, const RTC_GNSS_InitState& init) {
    const size_t used = strlen(out);
    if (used >= len) return;
    char sync_seq[24] = "null";
    char delay_us[24] = "null";
    char statistics_seq[24] = "null";
    if (init.statistics_start_pps_seq != 0) {
        snprintf(statistics_seq, sizeof(statistics_seq), "%lu",
                 (unsigned long)init.statistics_start_pps_seq);
    }
    if (init.rtc_synced_utc) {
        snprintf(sync_seq, sizeof(sync_seq), "%lu",
                 (unsigned long)init.rtc_sync_pps_seq);
        snprintf(delay_us, sizeof(delay_us), "%lu",
                 (unsigned long)init.rtc_sync_start_delay_us);
    }
    snprintf(out + used, len - used,
        ",\"utc_referencia_lista\":%s,"
        "\"rtc_sync_utc\":%s,"
        "\"rtc_sync_pps_seq\":%s,"
        "\"rtc_sync_inicio_post_pps_us\":%s,"
        "\"estadistica_inicio_pps_seq\":%s",
        init.utc_reference_ready ? "true" : "false",
        init.rtc_synced_utc ? "true" : "false",
        sync_seq, delay_us, statistics_seq);
}

void RTC_GNSS_Diagnostics_Module::buildJsonFields(
    char* out, size_t len, const RTC_GNSS_InitState& init,
    bool pps_usable, uint32_t seq, uint32_t ticks_raw,
    const Kalman_State* state, bool holdover_estimated) {
    if (len == 0) return;
    char rtc_json[40] = "null";
    char gnss_json[40] = "null";

    DateTime rtc_time(2000, 1, 1, 0, 0, 0);
    if (rtc_.readDateTime(rtc_time)) {
        char rtc_iso[32];
        RTC_Module::dateTimeToIsoLocal(rtc_time, rtc_iso, sizeof(rtc_iso));
        snprintf(rtc_json, sizeof(rtc_json), "\"%s\"", rtc_iso);
    }

    DateTime gnss_local(2000, 1, 1, 0, 0, 0);
    const bool gnss_ok =
        gps_.hasValidRmcRecent(GNSS_VALID_LOG_MAX_AGE_MS) &&
        gps_.getLastValidLocalDateTime(gnss_local);
    if (gnss_ok) {
        char gnss_iso[32];
        RTC_Module::dateTimeToIsoLocal(gnss_local, gnss_iso,
                                       sizeof(gnss_iso));
        snprintf(gnss_json, sizeof(gnss_json), "\"%s\"", gnss_iso);
    }
    snprintf(out, len,
        ",\"gnss_valido\":%s,\"rtc_hora\":%s,\"gnss_hora\":%s",
        gnss_ok ? "true" : "false", rtc_json, gnss_json);

    if (holdover_estimated) {
        // Sin nuevo PPS: no se compara contra GNSS, pero siguen el RTC 32k
        // y la correccion predicha de Kalman sobre el mismo origen.
        clocks_.appendHoldoverJson(out, len, ticks_raw, state);
    } else {
        clocks_.appendJson(out, len, pps_usable, seq, ticks_raw, state,
                           gnss_ok && gps_.hasValidRmcRecent(1500),
                           gnss_local, init.rtc_synced_utc,
                           init.rtc_sync_pps_seq, init.utc_reference_ready);
    }
    appendRmc(out, len, seq, gnss_ok, gnss_local);
    appendUbx(out, len);
    appendInit(out, len, init);
}