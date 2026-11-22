#include "Json_Log_Builder.h"

#include <RTClib.h>

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

namespace {

constexpr double TICK_US = 1000000.0 / 32768.0;

const char* typeName(JsonLogType type) {
    switch (type) {
        case JsonLogType::PPS:           return "PPS";
        case JsonLogType::HOLDOVER:      return "HOLDOVER";
        case JsonLogType::EVENTO_FISICO: return "EVENTO_FISICO";
    }
    return "DESCONOCIDO";
}

// Escritura incremental sobre un char[] fijo, sin String ni memoria dinamica.
// Escapa correctamente los valores de texto y verifica cada escritura.
class JsonWriter {
public:
    JsonWriter(char* output, size_t capacity)
        : out_(output), capacity_(capacity) {
        if (out_ == nullptr || capacity_ < 3) {
            ok_ = false;
            return;
        }
        out_[0] = '\0';
        appendChar('{');
    }

    void string(const char* key, const char* value) {
        field(key);
        if (value) quoted(value);
        else raw("null");
    }

    void boolean(const char* key, bool value) {
        field(key);
        raw(value ? "true" : "false");
    }

    void nullValue(const char* key) {
        field(key);
        raw("null");
    }

    void signedNumber(const char* key, int64_t value) {
        field(key);
        formatted("%lld", static_cast<long long>(value));
    }

    void unsignedNumber(const char* key, uint64_t value) {
        field(key);
        formatted("%llu", static_cast<unsigned long long>(value));
    }

    void decimal(const char* key, double value, int decimals = 6) {
        field(key);
        if (!isfinite(value)) raw("null");
        else formatted("%.*f", decimals, value);
    }

    bool finish() {
        appendChar('}');
        if (!ok_ && out_ != nullptr && capacity_ != 0) out_[0] = '\0';
        return ok_;
    }

private:
    char* out_ = nullptr;
    size_t capacity_ = 0;
    size_t used_ = 0;
    bool ok_ = true;
    bool first_ = true;

    void appendChar(char c) {
        if (!ok_) return;
        if (used_ + 1 >= capacity_) {
            ok_ = false;
            return;
        }
        out_[used_++] = c;
        out_[used_] = '\0';
    }

    void raw(const char* value) {
        if (!ok_ || !value) return;
        while (*value && ok_) appendChar(*value++);
    }

    void formatted(const char* fmt, ...) {
        if (!ok_) return;
        va_list args;
        va_start(args, fmt);
        const int n = vsnprintf(out_ + used_, capacity_ - used_, fmt, args);
        va_end(args);
        if (n < 0 || static_cast<size_t>(n) >= capacity_ - used_) {
            ok_ = false;
            return;
        }
        used_ += static_cast<size_t>(n);
    }

    void quoted(const char* value) {
        if (!value) { raw("null"); return; }
        appendChar('"');
        for (const unsigned char* p = reinterpret_cast<const unsigned char*>(value);
             *p && ok_; ++p) {
            switch (*p) {
                case '"': raw("\\\""); break;
                case '\\': raw("\\\\"); break;
                case '\n': raw("\\n"); break;
                case '\r': raw("\\r"); break;
                case '\t': raw("\\t"); break;
                default:
                    if (*p < 0x20) formatted("\\u%04x", static_cast<unsigned>(*p));
                    else appendChar(static_cast<char>(*p));
                    break;
            }
        }
        appendChar('"');
    }

    void field(const char* key) {
        if (!ok_) return;
        if (!first_) appendChar(',');
        first_ = false;
        quoted(key);
        appendChar(':');
    }
};

void addClockFields(JsonWriter& w, const ClockLogSnapshot& c) {
    if (c.has_ticks) w.unsignedNumber("sample_ticks", c.ticks);
    else w.nullValue("sample_ticks");

    if (c.gnss_ok) w.signedNumber("gnss_us", c.gnss_us);
    else w.nullValue("gnss_us");
    if (c.rtc_ok) w.signedNumber("rtc_us", c.rtc_us);
    else w.nullValue("rtc_us");
    if (c.rtc_raw_ok) w.signedNumber("rtc_fisico_us", c.rtc_raw_us);
    else w.nullValue("rtc_fisico_us");
    if (c.kalman_ok) w.signedNumber("kalman_us", c.kalman_us);
    else w.nullValue("kalman_us");

    if (c.gnss_ok && c.rtc_ok)
        w.signedNumber("offset_rtc_gnss_us", c.rtc_us - c.gnss_us);
    else w.nullValue("offset_rtc_gnss_us");

    if (c.gnss_ok && c.kalman_ok)
        w.signedNumber("offset_kalman_gnss_us", c.kalman_us - c.gnss_us);
    else w.nullValue("offset_kalman_gnss_us");

    if (c.rtc_ok && c.kalman_ok)
        w.signedNumber("kalman_menos_rtc_us", c.kalman_us - c.rtc_us);
    else w.nullValue("kalman_menos_rtc_us");
}

void addIsoClockTime(JsonWriter& w, const char* key,
                     bool ok, int64_t local_us) {
    if (!ok || local_us <= 0) {
        w.nullValue(key);
        return;
    }

    const uint32_t secs = uint32_t(local_us / 1000000LL);
    const uint32_t us = uint32_t(local_us % 1000000LL);
    const DateTime dt(secs);

    char formatted_time[40];
    snprintf(formatted_time, sizeof(formatted_time),
        "%04u-%02u-%02uT%02u:%02u:%02u.%06lu",
        unsigned(dt.year()), unsigned(dt.month()), unsigned(dt.day()),
        unsigned(dt.hour()), unsigned(dt.minute()), unsigned(dt.second()),
        (unsigned long)us);

    w.string(key, formatted_time);
}

void addKalmanFields(JsonWriter& w, const JsonLogRecord& r) {
    if (!r.has_kalman) {
        w.nullValue("kalman_phi");
        w.nullValue("kalman_f");
        w.nullValue("kalman_p00");
        w.nullValue("kalman_p01");
        w.nullValue("kalman_p10");
        w.nullValue("kalman_p11");
        w.nullValue("kalman_y_phi");
        w.nullValue("kalman_k_phi");
        w.nullValue("kalman_k_f");
        return;
    }

    w.decimal("kalman_phi", r.kalman.phi);
    w.decimal("kalman_f", r.kalman.f, 9);
    w.decimal("kalman_p00", r.kalman.p00);
    w.decimal("kalman_p01", r.kalman.p01);
    w.decimal("kalman_p10", r.kalman.p10);
    w.decimal("kalman_p11", r.kalman.p11);

    // Innovacion y ganancias son las del update, no las de un predict aislado.
    if (r.kalman_actualizado) {
        w.decimal("kalman_y_phi", r.kalman.y0);
        w.decimal("kalman_k_phi", r.kalman.k00);
        w.decimal("kalman_k_f", r.kalman.k10);
    } else {
        w.nullValue("kalman_y_phi");
        w.nullValue("kalman_k_phi");
        w.nullValue("kalman_k_f");
    }
}

}  // namespace

bool buildLogJson(const JsonLogRecord& r, char* out, size_t len) {
    if (!out || len == 0) return false;
    out[0] = '\0';
    if (len < 3) return false;
    if (!r.id || !r.id[0]) return false;

    JsonWriter w(out, len);
    w.unsignedNumber("schema", 2);
    w.string("id", r.id);  // El servidor.py usa esta clave para separar prototipos.
    w.string("modo", typeName(r.tipo));
    if (r.detalle && r.detalle[0]) w.string("detalle", r.detalle);
    w.unsignedNumber("ms", r.ms);
    w.signedNumber("aging", r.aging);
    w.boolean("rtc_aligned", r.rtc_aligned);
    w.boolean("kalman_trained", r.kalman_trained);
    w.boolean("rtc_calibrado", r.rtc_calibrated);
    if (r.rtc_calibrated)
        w.signedNumber("rtc_correccion_inicial_us", r.rtc_initial_correction_us);
    else w.nullValue("rtc_correccion_inicial_us");
    w.string("escala_hora", "LOCAL_UTC-3");

    addClockFields(w, r.clocks);

    if (r.tipo == JsonLogType::PPS) {
        if (r.has_pps) {
            w.boolean("pps_valid", r.pps.valid);
            w.unsignedNumber("pps_seq", r.pps.seq);
            w.unsignedNumber("pps_pending", r.pps.pending);
            w.unsignedNumber("pps_discarded", r.pps.discarded);
            w.unsignedNumber("pps_at_us", r.pps.at_us);
            w.unsignedNumber("pps_ticks", r.pps.ticks); // 64 bits; no truncar.
            w.unsignedNumber("pps_diff_ticks", r.pps.diff);
            w.unsignedNumber("pps_period_us", r.pps.period_us);
            if (r.pps.period_us > 0)
                w.signedNumber("pps_error_ticks", int64_t(r.pps.diff) - 32768LL);
            else w.nullValue("pps_error_ticks");
        } else {
            w.nullValue("pps_valid");
            w.nullValue("pps_seq");
            w.nullValue("pps_pending");
            w.nullValue("pps_discarded");
            w.nullValue("pps_at_us");
            w.nullValue("pps_ticks");
            w.nullValue("pps_diff_ticks");
            w.nullValue("pps_period_us");
            w.nullValue("pps_error_ticks");
        }
        w.boolean("interval_valid", r.interval_valid);
    }

    w.boolean("kalman_prediccion", r.kalman_prediccion);
    w.boolean("kalman_actualizado", r.kalman_actualizado);
    if (r.has_offset_ticks) {
        w.signedNumber("offset_ticks", r.offset_ticks);
        w.decimal("offset_us", r.offset_ticks * TICK_US, 3);
    } else {
        w.nullValue("offset_ticks");
        w.nullValue("offset_us");
    }
    addKalmanFields(w, r);

    if (r.tipo == JsonLogType::HOLDOVER) {
        w.unsignedNumber("ms_since_last_pps", r.ms_since_last_pps);
    } else if (r.tipo == JsonLogType::EVENTO_FISICO) {
        if (r.event_pps_age_valid)
            w.unsignedNumber("ms_since_last_pps", r.ms_since_last_pps);
        else w.nullValue("ms_since_last_pps");
    }
    if (r.tipo == JsonLogType::HOLDOVER) {
        w.unsignedNumber("holdover_steps", r.holdover_steps);
    }
    if (r.tipo == JsonLogType::EVENTO_FISICO) {
        w.boolean("gnss_evento_interpolado_32k", r.clocks.gnss_ok);
        w.unsignedNumber("evento_num", r.event_num);
        w.boolean("evento_captura_ok", r.event_captured);
        w.unsignedNumber("evento_esp_us", r.event_us);
        w.unsignedNumber("evento_retardo_registro_us", r.event_delay_to_log_us);
        w.unsignedNumber("evento_descartados", r.event_discarded);
    }

    if (r.gps_sats >= 0) w.signedNumber("satelites", r.gps_sats);
    else w.nullValue("satelites");
    if (r.gps_pdop >= 0) w.decimal("pdop", r.gps_pdop, 3);
    else w.nullValue("pdop");
    if (r.gps_hdop >= 0) w.decimal("hdop", r.gps_hdop, 3);
    else w.nullValue("hdop");
    if (r.has_utc_validation) w.boolean("utc_validada", r.utc_valid);
    else w.nullValue("utc_validada");
    
    addIsoClockTime(w, "gnss", r.clocks.gnss_ok, r.clocks.gnss_us);
    addIsoClockTime(w, "rtc", r.clocks.rtc_ok, r.clocks.rtc_us);
    addIsoClockTime(w, "kalman", r.clocks.kalman_ok, r.clocks.kalman_us);
    addIsoClockTime(w, "rtc_fisico", r.clocks.rtc_raw_ok, r.clocks.rtc_raw_us);

    return w.finish();
}
