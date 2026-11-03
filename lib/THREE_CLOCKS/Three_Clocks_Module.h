#ifndef THREE_CLOCKS_MODULE_H
#define THREE_CLOCKS_MODULE_H

#include <Arduino.h>
#include <RTClib.h>
#include "Kalman_Filter.h"

// Comparacion RELATIVA a partir de un PPS de origen.
// No modifica el DS3231, el contador PCNT ni el filtro de Kalman.
// La asociacion fisica entre una etiqueta RMC y el PPS no esta certificada.
class Three_Clocks_Module {
public:
    // Se llama con CADA intervalo PPS procesado, antes de construir el JSON.
    // Reconstruye pulsos ausentes solamente si micros() y los ticks 32k
    // respaldan el numero entero de segundos transcurridos.
    void observePpsInterval(bool coming_from_holdover,
                            uint32_t elapsed_us, uint32_t seq_delta,
                            uint32_t pending, uint32_t ticks_window);

    // Anexa exactamente los campos reloj_gnss_pps/reloj_rtc_32k/reloj_kalman
    // y sus desfases. Nuevos campos: segmento, recuperados y reanclajes.
    void appendJson(char* out, size_t len, bool pps_usable,
                    uint32_t seq, uint32_t ticks_raw,
                    const Kalman_State* kalman_state,
                    bool rmc_recent, const DateTime& gnss_local,
                    bool rtc_synced, uint32_t rtc_sync_pps_seq,
                    bool utc_reference_ready);

    // Durante HOLDOVER muestra las horas libres del RTC y de Kalman,
    // extrapoladas con el conteo PCNT actual. NO hay nueva referencia GNSS:
    // reloj_gnss_pps y los desfases frente a GNSS quedan en null.
    void appendHoldoverJson(char* out, size_t len,
                            uint32_t live_ticks_raw,
                            const Kalman_State* kalman_state);

    // Diagnostico RMC: desfase entre la etiqueta y el segundo PPS reconstruido.
    bool differenceFromRmc(uint32_t seq, bool rmc_recent,
                           const DateTime& rmc_local, int64_t& diff_s) const;

private:
    struct Base {
        bool available = false;
        uint32_t seq_start = 0;
        uint32_t ticks_start = 0;
        uint32_t epoch_start = 0;
        uint32_t start_ms = 0;
        double phi_start = 0.0;
    } base_;

    uint32_t recovered_seconds_ = 0;
    uint32_t segment_ = 0;
    uint32_t reanchors_ = 0;
    bool reference_trusted_ = true;
    uint8_t clean_reanchor_samples_ = 0;
    uint32_t reanchor_prev_seq_ = 0;
    uint32_t reanchor_prev_epoch_ = 0;

    bool elapsedSeconds(uint32_t seq, uint32_t& seconds) const;
    void establishBase(uint32_t seq, uint32_t ticks_raw,
                       uint32_t epoch, double phi);
    void trackReanchor(uint32_t seq, uint32_t ticks_raw,
                       const DateTime& gnss_local,
                       const Kalman_State& kalman_state);
    static void formatLocalTimeUs(int64_t timestamp_us, char* out, size_t len);
    static int64_t roundMicroseconds(double value);
};

#endif
