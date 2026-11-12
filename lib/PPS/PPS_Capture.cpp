#include "PPS_Capture.h"
#include "Capture_Instances.h"

namespace PPS_Capture {

static constexpr uint32_t PPS_MIN_GAP_US = 500000UL;

void begin(int pin_rtc_32k, int pin_gnss_pps) {
    if (!CaptureInstances::pcnt().begin(pin_rtc_32k)) {
        Serial.println("ERROR: no se pudo iniciar PCNT extendido");
        while (true) delay(1000);
    }
    if (!CaptureInstances::pps().begin(pin_gnss_pps, RISING,
                                       PPS_MIN_GAP_US, INPUT)) {
        Serial.println("ERROR: no se pudo iniciar captura PPS");
        while (true) delay(1000);
    }
}

bool consumePps(PpsSample& sample) {
    TickEdgeSample edge;
    if (!CaptureInstances::pps().consume(edge)) return false;
    // La API original recibe uint32_t; queda módulo 2^32 como antes.
    sample.ticks_raw = static_cast<uint32_t>(edge.ticks);
    sample.seq = edge.seq;
    sample.pending = edge.pending;
    sample.pps_time_us = edge.at_us;
    sample.pps_period_us = edge.period_us;
    return true;
}

void snapshotLastPps(uint32_t& seq, uint32_t& pps_time_us) {
    TickEdgeSample edge;
    if (!CaptureInstances::pps().snapshot(edge)) {
        seq = 0;
        pps_time_us = 0;
        return;
    }
    seq = edge.seq;
    pps_time_us = edge.at_us;
}

bool snapshotCurrentTicks(uint32_t& ticks_raw) {
    if (!CaptureInstances::pcnt().started()) return false;
    const uint32_t current = static_cast<uint32_t>(CaptureInstances::pcnt().read());
    TickEdgeSample last;
    if (CaptureInstances::pps().snapshot(last)) {
        if (static_cast<int32_t>(current - static_cast<uint32_t>(last.ticks)) < 0) {
            return false;
        }
    }
    ticks_raw = current;
    return true;
}

}