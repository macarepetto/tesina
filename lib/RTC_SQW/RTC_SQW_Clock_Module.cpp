#include "RTC_SQW_Clock_Module.h"

#include <Wire.h>
#include <stdio.h>
#include <string.h>

#include "RTC32K_Pcnt_Capture_Module.h"
#include "RTC_Module.h"
#include "RTC_SQW_Time_Math.h"
#include "Telemetry_Queue_Module.h"

namespace RTC_SQW_Clock {
namespace {

constexpr uint8_t DS3231_I2C_ADDRESS = 0x68;
constexpr uint8_t DS3231_CONTROL_REGISTER = 0x0E;
constexpr uint8_t SQW_MODE_MASK = 0x1C;  // INTCN, RS2 y RS1
constexpr uint32_t MAX_CALENDAR_READ_DELAY_US = 250000U;
constexpr int32_t MAX_TICK_ERROR_PER_RTC_SECOND = 10;

struct SqwSample {
    uint32_t seq = 0;
    uint32_t ticks_raw = 0;
    uint32_t at_us = 0;
    uint32_t discarded = 0;
    bool capture_ok = false;
};

portMUX_TYPE sqw_mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool sqw_pending = false;
volatile uint32_t sqw_seq = 0;
volatile uint32_t sqw_ticks_raw = 0;
volatile uint32_t sqw_at_us = 0;
volatile uint32_t sqw_discarded = 0;
volatile bool sqw_capture_ok = false;

bool started = false;
uint32_t reject_through_seq = 0;
uint32_t last_good_seq = 0;
uint32_t last_good_ticks = 0;
int64_t last_good_epoch_s = 0;

bool anchor_valid = false;
uint32_t anchor_ticks = 0;
int64_t anchor_epoch_s = 0;

void IRAM_ATTR onSqw() {
    // Usa el MISMO PCNT del PPS; lectura atomica, sin resetear el contador.
    RTC32K_Pcnt_Capture::ExternalEventSample capture;
    const bool captured =
        RTC32K_Pcnt_Capture::captureExternalEventFromIsr(capture);

    portENTER_CRITICAL_ISR(&sqw_mux);
    ++sqw_seq;
    if (sqw_pending) ++sqw_discarded;
    sqw_pending = true;
    sqw_ticks_raw = capture.ticks_raw;
    sqw_at_us = capture.event_micros;
    sqw_capture_ok = captured;
    portEXIT_CRITICAL_ISR(&sqw_mux);
}

bool consumeSqw(SqwSample& sample) {
    bool available = false;
    portENTER_CRITICAL(&sqw_mux);
    if (sqw_pending) {
        sample.seq = sqw_seq;
        sample.ticks_raw = sqw_ticks_raw;
        sample.at_us = sqw_at_us;
        sample.discarded = sqw_discarded;
        sample.capture_ok = sqw_capture_ok;
        sqw_pending = false;
        available = true;
    }
    portEXIT_CRITICAL(&sqw_mux);
    return available;
}

uint32_t currentSqwSeq() {
    portENTER_CRITICAL(&sqw_mux);
    const uint32_t seq = sqw_seq;
    portEXIT_CRITICAL(&sqw_mux);
    return seq;
}

bool setDs3231Sqw1Hz() {
    // Preserva los demas bits de control del DS3231; desactiva modo
    // interrupcion de alarmas y selecciona la salida SQW de 1 Hz.
    Wire.beginTransmission(DS3231_I2C_ADDRESS);
    Wire.write(DS3231_CONTROL_REGISTER);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(DS3231_I2C_ADDRESS, (uint8_t)1) != 1) {
        return false;
    }
    const uint8_t control = (uint8_t)Wire.read();
    const uint8_t new_control = (uint8_t)(control & ~SQW_MODE_MASK);

    Wire.beginTransmission(DS3231_I2C_ADDRESS);
    Wire.write(DS3231_CONTROL_REGISTER);
    Wire.write(new_control);
    return Wire.endTransmission() == 0;
}

void formatEpochUs(int64_t epoch_us, char* out, size_t len) {
    if (!out || len == 0) return;
    if (epoch_us < 0) {
        snprintf(out, len, "SIN_FECHA");
        return;
    }
    const uint32_t epoch_s = (uint32_t)(epoch_us / 1000000LL);
    const uint32_t fraction_us = (uint32_t)(epoch_us % 1000000LL);
    const DateTime dt(epoch_s);
    snprintf(out, len, "%04u-%02u-%02uT%02u:%02u:%02u.%06lu",
             (unsigned)dt.year(), (unsigned)dt.month(),
             (unsigned)dt.day(), (unsigned)dt.hour(),
             (unsigned)dt.minute(), (unsigned)dt.second(),
             (unsigned long)fraction_us);
}

}  // namespace

bool begin(int pin_rtc_sqw) {
    if (started) return true;
    if (pin_rtc_sqw < 0) return false;
    if (!setDs3231Sqw1Hz()) return false;

    portENTER_CRITICAL(&sqw_mux);
    sqw_pending = false;
    sqw_seq = 0;
    sqw_discarded = 0;
    portEXIT_CRITICAL(&sqw_mux);

    anchor_valid = false;
    last_good_seq = 0;
    pinMode(pin_rtc_sqw, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pin_rtc_sqw), onSqw, FALLING);
    started = true;
    return true;
}

void invalidateOnCalendarWrite() {
    // No altera PCNT, PPS ni Kalman. Solo invalida el origen propio RTC.
    anchor_valid = false;
    last_good_seq = 0;
    reject_through_seq = currentSqwSeq();
}

bool timeAtTicks(uint32_t event_ticks_raw, int64_t& epoch_us,
                 uint32_t& ticks_since_sqw) {
    if (!anchor_valid) return false;
    return RTC_SQW_Time_Math::convert(
        anchor_ticks, event_ticks_raw, anchor_epoch_s,
        epoch_us, ticks_since_sqw);
}

void service(RTC_Module& rtc, const char* device_id,
             Telemetry_Queue_Module* telemetry) {
    if (!started) return;

    SqwSample sample;
    if (!consumeSqw(sample)) return;
    if (sample.seq <= reject_through_seq) return;

    const uint32_t capture_age_us = (uint32_t)(micros() - sample.at_us);
    const char* state = "RTC_SQW_INICIO";
    uint32_t window_ticks = 0;
    int64_t epoch_s = 0;
    bool valid = sample.capture_ok;

    if (!sample.capture_ok) {
        state = "RTC_SQW_PCNT_FALLO";
    } else if (capture_age_us > MAX_CALENDAR_READ_DELAY_US) {
        // El calendario podria haber cambiado desde la interrupcion.
        valid = false;
        state = "RTC_SQW_LECTURA_TARDIA";
    } else if (!rtc.readEpoch(epoch_s)) {
        valid = false;
        state = "RTC_SQW_CALENDARIO_INVALIDO";
    } else if (sample.seq != currentSqwSeq()) {
        // Entro un nuevo SQW mientras se leia I2C.
        valid = false;
        state = "RTC_SQW_CAMBIO_DURANTE_LECTURA";
    }

    if (valid && last_good_seq > 0 && sample.seq == last_good_seq + 1) {
        window_ticks = sample.ticks_raw - last_good_ticks;
        const int64_t tick_error =
            (int64_t)window_ticks - RTC_SQW_Time_Math::TICKS_PER_SECOND;

        if (epoch_s != last_good_epoch_s + 1LL) {
            valid = false;
            state = "RTC_SQW_SEGUNDOS_NO_CONSECUTIVOS";
        } else if (tick_error > MAX_TICK_ERROR_PER_RTC_SECOND ||
                   tick_error < -MAX_TICK_ERROR_PER_RTC_SECOND) {
            valid = false;
            state = "RTC_SQW_TICKS_INCONSISTENTES";
        } else {
            state = "RTC_SQW_OK";
        }
    } else if (valid && last_good_seq > 0) {
        state = "RTC_SQW_REANCLA_TRAS_SALTO";
    }

    if (valid) {
        // Se ancla a la FECHA/HORA DEL RTC (I2C), NO a la fecha GNSS.
        anchor_ticks = sample.ticks_raw;
        anchor_epoch_s = epoch_s;
        anchor_valid = true;
        last_good_seq = sample.seq;
        last_good_ticks = sample.ticks_raw;
        last_good_epoch_s = epoch_s;
    } else {
        anchor_valid = false;
        last_good_seq = 0;
    }

    // El main solo requiere el anclaje para incluir el cuarto reloj
    // en el JSON normal, sin generar una segunda linea cada segundo.
    if (telemetry == nullptr) return;

    char time_iso[48] = "";
    if (valid) {
        formatEpochUs(epoch_s * 1000000LL, time_iso, sizeof(time_iso));
    }
    char json[384];
    snprintf(json, sizeof(json),
             "{\"device\":\"%s\",\"modo\":\"RTC_SQW_SEGUNDO\","
             "\"sqw_seq\":%lu,\"sqw_ticks_raw\":%lu,"
             "\"sqw_ticks_entre_segundos\":%lu,"
             "\"sqw_edad_lectura_us\":%lu,"
             "\"sqw_descartados\":%lu,"
             "\"reloj_rtc_calendario_sqw\":%s%s%s,"
             "\"sqw_estado\":\"%s\"}",
             device_id ? device_id : "desconocido",
             (unsigned long)sample.seq,
             (unsigned long)sample.ticks_raw,
             (unsigned long)window_ticks,
             (unsigned long)capture_age_us,
             (unsigned long)sample.discarded,
             valid ? "\"" : "", valid ? time_iso : "null",
             valid ? "\"" : "", state);
    telemetry->enqueue(json);
}

void service(RTC_Module& rtc, const char* device_id,
             Telemetry_Queue_Module& telemetry) {
    service(rtc, device_id, &telemetry);
}

void appendClockJson(char* out, size_t len, bool ticks_disponibles,
                     uint32_t ticks_raw) {
    if (out == nullptr || len == 0) return;
    const size_t used = strlen(out);
    if (used >= len) return;

    int64_t epoch_us = 0;
    uint32_t ticks_since_sqw = 0;
    if (!ticks_disponibles ||
        !timeAtTicks(ticks_raw, epoch_us, ticks_since_sqw)) {
        snprintf(out + used, len - used, ",\"reloj_rtc_calendario_sqw\":null");
        return;
    }

    char iso[48];
    formatEpochUs(epoch_us, iso, sizeof(iso));
    snprintf(out + used, len - used,
             ",\"reloj_rtc_calendario_sqw\":\"%s\"", iso);
}

void publishAtPps(const char* device_id, uint32_t pps_seq,
                  uint32_t pps_ticks_raw,
                  Telemetry_Queue_Module& telemetry) {
    if (!started) return;

    int64_t epoch_us = 0;
    uint32_t ticks_since_sqw = 0;
    const bool valid = timeAtTicks(pps_ticks_raw, epoch_us,
                                   ticks_since_sqw);
    char time_iso[48] = "";
    if (valid) formatEpochUs(epoch_us, time_iso, sizeof(time_iso));

    char ticks_json[24] = "null";
    if (valid) {
        snprintf(ticks_json, sizeof(ticks_json), "%lu",
                 (unsigned long)ticks_since_sqw);
    }

    char json[320];
    snprintf(json, sizeof(json),
             "{\"device\":\"%s\",\"modo\":\"RTC_SQW_EN_PPS\","
             "\"pps_seq\":%lu,\"pps_ticks_raw\":%lu,"
             "\"reloj_rtc_calendario_sqw\":%s%s%s,"
             "\"sqw_ticks_desde_inicio_segundo\":%s,"
             "\"sqw_estado\":\"%s\"}",
             device_id ? device_id : "desconocido",
             (unsigned long)pps_seq, (unsigned long)pps_ticks_raw,
             valid ? "\"" : "", valid ? time_iso : "null",
             valid ? "\"" : "",
             ticks_json,
             valid ? "RTC_SQW_OK" : "SIN_ANCLA_SQW");
    telemetry.enqueue(json);
}

}  // namespace RTC_SQW_Clock
