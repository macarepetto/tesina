#include "Clock_GNSS.h"

void ClockGNSS::establishBase(uint32_t seq, uint64_t ticks, uint32_t epoch) {
    base_.available = true;
    base_.seq_start = seq;
    base_.ticks_start = ticks;
    base_.epoch_start = epoch;
    base_.start_ms = millis();
    recovered_seconds_ = 0;
    reference_trusted_ = true;
    clean_reanchor_samples_ = 0;
    ++segment_;
}

bool ClockGNSS::elapsedSeconds(uint32_t seq, uint32_t& seconds) const {
    if (!base_.available || seq < base_.seq_start) return false;
    const uint64_t total = uint64_t(seq - base_.seq_start) +
                           recovered_seconds_;
    if (total >= MAX_SEGMENT_SECONDS) return false;
    seconds = uint32_t(total);
    return true;
}

void ClockGNSS::observePpsInterval(
    bool coming_from_holdover, uint32_t elapsed_us,
    uint32_t seq_delta, uint32_t pending, uint32_t ticks_window) {
    if (!base_.available || !coming_from_holdover) return;

    // Trasladado de Three_Clocks_Module::observePpsInterval(), sin cambios
    // en la tolerancia ni en las condiciones de reconstrucción.
    const uint32_t rounded = uint32_t((uint64_t(elapsed_us) + 500000ULL) /
                                       1000000ULL);
    const uint64_t reference_us = uint64_t(rounded) * 1000000ULL;
    const uint64_t error_us = elapsed_us >= reference_us
        ? uint64_t(elapsed_us) - reference_us
        : reference_us - uint64_t(elapsed_us);
    const int64_t ticks_expected = int64_t(rounded) * 32768LL;
    const int64_t ticks_error = int64_t(ticks_window) - ticks_expected;
    const int64_t tick_tolerance = 10LL * rounded;

    if (pending == 1 && seq_delta == 1 && rounded >= 2 &&
        error_us <= PPS_INTERVAL_TOLERANCE_US &&
        ticks_error >= -tick_tolerance && ticks_error <= tick_tolerance &&
        recovered_seconds_ <= MAX_SEGMENT_SECONDS - rounded) {
        recovered_seconds_ += rounded - seq_delta;
    } else if (rounded > 1 || pending != 1 || seq_delta != 1) {
        reference_trusted_ = false;
        clean_reanchor_samples_ = 0;
    }
}

bool ClockGNSS::differenceFromRmc(
    uint32_t seq, bool rmc_recent, const DateTime& local,
    int64_t& diff_s) const {
    uint32_t seconds = 0;
    if (!rmc_recent || !elapsedSeconds(seq, seconds)) return false;
    diff_s = int64_t(local.unixtime()) -
             (int64_t(base_.epoch_start) + seconds);
    return true;
}

void ClockGNSS::trackReanchor(uint32_t seq, uint64_t ticks,
                              const DateTime& local) {
    const uint32_t epoch = local.unixtime();
    if (clean_reanchor_samples_ > 0 &&
        seq == reanchor_prev_seq_ + 1 &&
        epoch == reanchor_prev_epoch_ + 1) {
        ++clean_reanchor_samples_;
    } else {
        clean_reanchor_samples_ = 1;
    }
    reanchor_prev_seq_ = seq;
    reanchor_prev_epoch_ = epoch;

    if (clean_reanchor_samples_ >= 3) {
        establishBase(seq, ticks, epoch);
        ++reanchors_;
    }
}

bool ClockGNSS::observePps(bool pps_usable, uint32_t seq, uint64_t ticks,
                           const Kalman_State* state, bool rmc_recent,
                           const DateTime& local, bool rtc_synced,
                           uint32_t rtc_sync_pps_seq,
                           bool utc_reference_ready) {
    last_pps_usable_ = false;
    reanchored_now_ = false;

    // Misma referencia inicial que Three_Clocks_Module::appendJson().
    if (!base_.available && pps_usable && rtc_synced &&
        rtc_sync_pps_seq == seq && state && rmc_recent) {
        establishBase(seq, ticks, local.unixtime());
    }

    const bool sample_ready = pps_usable && state && rmc_recent;
    uint32_t seconds = 0;
    const bool old_base_usable = elapsedSeconds(seq, seconds);

    if (base_.available && reference_trusted_ && sample_ready) {
        int64_t rmc_difference = 0;
        const bool rmc_mismatch = sample_ready && old_base_usable &&
            differenceFromRmc(seq, rmc_recent, local, rmc_difference) &&
            rmc_difference != 0;
        if (!old_base_usable || rmc_mismatch) {
            reference_trusted_ = false;
            clean_reanchor_samples_ = 0;
        }
    }

    if (base_.available && !reference_trusted_) {
        if (sample_ready && utc_reference_ready) {
            const uint32_t previous_reanchors = reanchors_;
            trackReanchor(seq, ticks, local);
            reanchored_now_ = reanchors_ != previous_reanchors;
        } else {
            clean_reanchor_samples_ = 0;
        }
    }

    const bool usable = sample_ready && reference_trusted_ &&
                        elapsedSeconds(seq, seconds);
    if (!usable) return false;

    last_pps_ticks_ = ticks;
    last_pps_epoch_us_ =
        (int64_t(base_.epoch_start) + seconds) * 1000000LL;
    last_pps_usable_ = true;
    return true;
}

bool ClockGNSS::getTimeAtTicks(uint64_t ticks, int64_t& time_us) {
    if (!last_pps_usable_ || ticks < last_pps_ticks_ ||
        ticks - last_pps_ticks_ > MAX_EDGE_AGE_TICKS) return false;
    // En el propio flanco PPS la fracción GNSS es 0.
    // Entre flancos es interpolación nominal basada en PCNT/RTC.
    time_us = last_pps_epoch_us_ + ticksToUs(ticks - last_pps_ticks_);
    return true;
}
