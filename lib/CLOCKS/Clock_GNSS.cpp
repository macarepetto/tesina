#include "Clock_GNSS.h"

bool ClockGNSS::update() {
    TickEdgeSample edge;
    if (!pps_.peek(edge) || !edge.valid || edge.seq == 0) return false;
    // El primer intervalo y el primer flanco tras un intervalo largo NO
    // sirven para anclar UTC; respetamos el descarte de CAPTURE_CORE.
    if (!edge.diff || edge.diff > MAX_EDGE_AGE_TICKS ||
        edge.period_us < 800000UL || edge.period_us > 1200000UL) return false;
    if (has_reference_ && edge.ticks == ref_ticks_) return false;

    GpsUtcSample utc;
    if (!gps_.getValidatedUtcSample(utc)) return false;
    if (utc.rmc_revision == 0) return false;

    // u-blox 6 a 1 Hz: se espera la sentencia RMC del ciclo DESPUES
    // del PPS al que se refiere. Las mediciones de tiempo son del
    // procesamiento UART (no timestamp hardware): comprobacion operativa,
    // NO certificado de precision de fase absoluta.
    const uint32_t inicio = uint32_t(utc.rmc_inicio_us - edge.at_us);
    const uint32_t fin    = uint32_t(utc.rmc_fin_us - edge.at_us);
    if (inicio > 850000UL || fin > 950000UL || fin < inicio) return false;

    // Revalidar: pudo llegar un PPS mas nuevo mientras lei la UART.
    TickEdgeSample current;
    if (!pps_.peek(current) || current.seq != edge.seq ||
        current.ticks != edge.ticks ||
        uint32_t(micros() - edge.at_us) > 950000UL) return false;

    // Si dos referencias son consecutivas, su etiqueta debe avanzar
    // exactamente un segundo. No atribuir automaticamente +/-1 segundo.
    if (has_reference_ && edge.ticks > ref_ticks_) {
        const uint64_t dt = edge.ticks - ref_ticks_;
        if (dt >= 30000ULL && dt <= 35500ULL &&
            utc.local_epoch_s != ref_time_us_ / 1000000LL + 1LL) return false;
    }

    ref_ticks_ = edge.ticks;
    ref_seq_ = edge.seq;
    ref_time_us_ = utc.local_epoch_s * 1000000LL;
    has_reference_ = true;
    return true;
}

bool ClockGNSS::lastReference(uint64_t& ticks, int64_t& local_epoch_s) const {
    if (!has_reference_) return false;
    ticks = ref_ticks_;
    local_epoch_s = ref_time_us_ / 1000000LL;
    return true;
}

bool ClockGNSS::getTimeAtTicks(uint64_t ticks, int64_t& time_us) {
    if (!has_reference_) return false;
    TickEdgeSample edge;
    if (!pps_.peek(edge) || !edge.valid || edge.seq == 0 ||
        edge.seq != ref_seq_ || edge.ticks != ref_ticks_) return false;
    if (ticks < ref_ticks_ || ticks - ref_ticks_ > MAX_EDGE_AGE_TICKS) return false;
    time_us = ref_time_us_ + ticksToUs(ticks - ref_ticks_);
    return true;
}
