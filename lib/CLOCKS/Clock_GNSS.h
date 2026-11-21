#pragma once
#include <Arduino.h>
#include <RTClib.h>
#include "Clock.h"
#include "Kalman_Filter.h"

// Reloj GNSS construido con la etiqueta RMC validada y la secuencia PPS.
// Port de la parte GNSS de Three_Clocks_Module; no lee I2C,
// no modifica Kalman, no consume capturas PPS ni usa la hora del RTC.
class ClockGNSS : public Clock {
public:
    explicit ClockGNSS(PcntExtended& counter) : Clock(counter) {}

    // Misma recuperación de segundos que Three_Clocks_Module.
    void observePpsInterval(bool coming_from_holdover,
                            uint32_t elapsed_us, uint32_t seq_delta,
                            uint32_t pending, uint32_t ticks_window);

    // Misma asociación, detección de discrepancias y reanclaje que
    // Three_Clocks_Module::appendJson(). Se ejecuta DESPUES del
    // procesamiento PPS en main, sin añadir otras validaciones GNSS.
    bool observePps(bool pps_usable, uint32_t seq, uint64_t ticks,
                    const Kalman_State* state, bool rmc_recent,
                    const DateTime& gnss_local, bool rtc_synced,
                    uint32_t rtc_sync_pps_seq, bool utc_reference_ready);

    // En un PPS devuelve el segundo GNSS entero. Entre PPS interpola
    // la fracción con PCNT/32 kHz; esa interpolación NO es una nueva
    // medición GNSS ni certifica fase UTC de submicrosegundos.
    bool getTimeAtTicks(uint64_t ticks, int64_t& time_us) override;

    bool differenceFromRmc(uint32_t seq, bool rmc_recent,
                           const DateTime& rmc_local, int64_t& diff_s) const;

    uint32_t segment() const { return segment_; }
    uint32_t recoveredSeconds() const { return recovered_seconds_; }
    uint32_t reanchors() const { return reanchors_; }
    bool trusted() const { return reference_trusted_; }
    bool reanchoredAtLastPps() const { return reanchored_now_; }

private:
    static constexpr uint32_t MAX_SEGMENT_SECONDS = 64800UL;
    static constexpr uint32_t PPS_INTERVAL_TOLERANCE_US = 200000UL;
    struct Base {
        bool available = false;
        uint32_t seq_start = 0;
        uint64_t ticks_start = 0;
        uint32_t epoch_start = 0;
        uint32_t start_ms = 0;
    } base_;

    uint32_t recovered_seconds_ = 0;
    uint32_t segment_ = 0;
    uint32_t reanchors_ = 0;
    bool reference_trusted_ = true;
    uint8_t clean_reanchor_samples_ = 0;
    uint32_t reanchor_prev_seq_ = 0;
    uint32_t reanchor_prev_epoch_ = 0;

    bool last_pps_usable_ = false;
    bool reanchored_now_ = false;
    uint64_t last_pps_ticks_ = 0;
    int64_t last_pps_epoch_us_ = 0;

    bool elapsedSeconds(uint32_t seq, uint32_t& seconds) const;
    void establishBase(uint32_t seq, uint64_t ticks, uint32_t epoch);
    void trackReanchor(uint32_t seq, uint64_t ticks,
                       const DateTime& local);
};
