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

//DEBERIA IR EN OTRO LADO
static void showTime(const char* name, bool ok, int64_t local_us) {
    if (!ok || local_us <= 0) {
        Serial.printf("%-12s: NO DISPONIBLE\n", name);
        return;
    }

    const uint32_t secs = uint32_t(local_us / 1000000LL);
    const uint32_t us = uint32_t(local_us % 1000000LL);

    const DateTime t(secs);

    Serial.printf(
        "%-12s: %04u-%02u-%02u %02u:%02u:%02u.%06lu\n",
        name,
        unsigned(t.year()),
        unsigned(t.month()),
        unsigned(t.day()),
        unsigned(t.hour()),
        unsigned(t.minute()),
        unsigned(t.second()),
        (unsigned long)us
    );
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

    // UNA lectura del PCNT para comparar exactamente el mismo instante.
    static uint32_t ultimo_print_ms = 0;
    if (ms - ultimo_print_ms >= 250UL) {
        ultimo_print_ms = ms;
        const uint64_t same_ticks = app.pcnt().read();
        int64_t gnss = 0, rtc = 0, kalman = 0;
        const bool gnss_ok = app.clockGnss().getTimeAtTicks(same_ticks, gnss);
        const bool rtc_ok = app.clockRtc().getTimeAtTicks(same_ticks, rtc);
        const bool kalman_ok = rtc_aligned && kalman_trained &&
                         app.clockKalman().getTimeAtTicks(same_ticks, kalman);
        Serial.println("-----------------------------");
        showTime("GNSS/PPS", gnss_ok, gnss);
        showTime("RTC/SQW", rtc_ok, rtc);
        showTime("RTC+KALMAN", kalman_ok, kalman);
        Serial.printf("RTC sincronizado: %s | Kalman entrenado: %s | phi: %.3f ticks\n",
                      rtc_aligned ? "SI" : "NO", kalman_trained ? "SI" : "NO",
                      app.kalman().getState().phi);
        if (gnss_ok && rtc_ok) {
            Serial.printf("offset RTC-GNSS: %lld us\n", (long long)(rtc - gnss));
        }
        if (gnss_ok && kalman_ok) {
            Serial.printf("offset Kalman-GNSS: %lld us\n", (long long)(kalman - gnss));
        }
    }
    delay(100);
}