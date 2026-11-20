#include <Arduino.h>
#include <stdlib.h>
#include <RTClib.h>
#include "Setup.h"

static constexpr int32_t MAX_PPS_ERROR_TICKS = 10;

Setup app("esp32-prototipo-1");

bool rtc_aligned = false;
bool kalman_trained = false;
int64_t offset_ticks = 0;
uint32_t last_pps_ms = 0;
uint32_t last_predict_ms = 0;

// Misma conversion de fecha/hora que utilizaba showTime(), ahora para JSON.
static void formatTimeJson(char* out, size_t size, bool ok, int64_t local_us) {
    if (!ok || local_us <= 0) {
        snprintf(out, size, "null");
        return;
    }

    const uint32_t secs = uint32_t(local_us / 1000000LL);
    const uint32_t us = uint32_t(local_us % 1000000LL);
    const DateTime t(secs);

    snprintf(out, size,
        "\"%04u-%02u-%02uT%02u:%02u:%02u.%06lu\"",
        unsigned(t.year()),
        unsigned(t.month()),
        unsigned(t.day()),
        unsigned(t.hour()),
        unsigned(t.minute()),
        unsigned(t.second()),
        (unsigned long)us);
}

void setup() {
    app.begin(); // Serial, RTC, aging, GNSS, PCNT, PPS, SQW y reset de Kalman.
    Serial.println("Las 3 horas se consultan a ticks iguales. RTC se ajustara una vez desde GNSS real.");
    Serial.println("Verificar en el receptor la asociacion RMC/PPS.");
}

void loop() {
    // La UART y los relojes se atienden continuamente, fuera de ISR.
    app.gps().update();
    app.gps().consultarUbxUtcSiCorresponde();
    app.clockRtc().update(); // UNICO consumidor de SQW.

    TickEdgeSample edge;
    if (app.pps().consume(edge)) { // UNICO consumidor de PPS; GNSS solo usa peek().
        last_pps_ms = millis();
        last_predict_ms = last_pps_ms;
        if (!rtc_aligned) {
            // La RMC del PPS anterior ya fue validada y vinculada a ese flanco.
            // Al llegar el SIGUIENTE PPS (consecutivo) avanzamos un segundo.
            uint64_t prev_ticks = 0;
            int64_t prev_epoch = 0;
            if (app.clockGnss().lastReference(prev_ticks, prev_epoch) &&
                edge.ticks > prev_ticks &&
                edge.ticks - prev_ticks >= 32768ULL - MAX_PPS_ERROR_TICKS &&
                edge.ticks - prev_ticks <= 32768ULL + MAX_PPS_ERROR_TICKS &&
                uint32_t(micros() - edge.at_us) <= 50000UL) {
                const DateTime nextSecond(uint32_t(prev_epoch + 1));
                if (app.rtc().setFromDateTime(nextSecond)) {
                    app.clockRtc().invalidate();
                    app.kalman().reset();
                    app.clockKalman().setReference();
                    offset_ticks = 0;
                    rtc_aligned = true;
                    kalman_trained = false;
                    Serial.println("RTC ajustado una vez en PPS siguiente, con UTC/GNSS validada.");
                }
            }
        } else {
            // Medicion DIRECTA entre flancos PPS desde CAPTURE_CORE.
            const int64_t error = int64_t(edge.diff) - 32768LL;
            const bool intervalo_valido = edge.valid && edge.seq != 0 &&
                edge.pending == 1 &&
                edge.period_us >= 800000UL && edge.period_us <= 1200000UL &&
                llabs((long long)error) <= MAX_PPS_ERROR_TICKS;
            if (intervalo_valido) {
                offset_ticks += error;
                app.kalman().predict();
                app.kalman().updatePhi(double(offset_ticks));
                kalman_trained = true;
            }
        }
    }
    // RMC de este ciclo llega DESPUES del PPS; la clase asocia al PPS con peek().
    app.clockGnss().update();

    // En perdida de PPS, solo predict (no actualizar con SQW).
    const uint32_t ms = millis();
    if (rtc_aligned && ms - last_pps_ms > 1500UL &&
        ms - last_predict_ms >= 1000UL) {
        app.kalman().predict();
        last_predict_ms = ms;
    }

    // Cada 1 segundo: las tres horas consultadas sobre una sola lectura del PCNT.
    static uint32_t ultimo_log_ms = 0;
    if (ms - ultimo_log_ms >= 1000UL) {
        ultimo_log_ms = ms;
        const uint64_t same_ticks = app.pcnt().read();
        int64_t gnss = 0, rtc = 0, kalman = 0;
        const bool gnss_ok = app.clockGnss().getTimeAtTicks(same_ticks, gnss);
        const bool rtc_ok = app.clockRtc().getTimeAtTicks(same_ticks, rtc);
        const bool kalman_ok = rtc_aligned && kalman_trained &&
                         app.clockKalman().getTimeAtTicks(same_ticks, kalman);

        // Sin hora disponible, el campo correspondiente se envia como null.
        char gnss_json[40], rtc_json[40], kalman_json[40];
        formatTimeJson(gnss_json, sizeof(gnss_json), gnss_ok, gnss);
        formatTimeJson(rtc_json, sizeof(rtc_json), rtc_ok, rtc);
        formatTimeJson(kalman_json, sizeof(kalman_json), kalman_ok, kalman);

        char json[256];
        const int n = snprintf(json, sizeof(json),
            "{\"id\":\"%s\",\"gnss\":%s,\"rtc\":%s,\"kalman\":%s}",
            app.deviceId(), gnss_json, rtc_json, kalman_json);

        if (n > 0 && n < (int)sizeof(json) && app.wifi().isServerConnected()) {
            app.telemetryQueue().enqueue(json);
        }
    }

    delay(2);
}