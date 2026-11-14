#pragma once
#include "Clock.h"
#include "Tick_Capture.h"
#include "GPS_Module.h"

// GNSS = etiqueta RMC corroborada con UBX + flanco PPS del mismo ciclo.
// No consume PPS: CAPTURE_CORE puede entregar esa muestra a Kalman.
class ClockGNSS : public Clock {
public:
    ClockGNSS(PcntExtended& counter, PeriodicTickCapture& pps, GPS_Module& gps)
        : Clock(counter), pps_(pps), gps_(gps) {}

    // Invocar desde loop(), DESPUES de gps.update(); NUNCA en ISR.
    // Devuelve true solo al aceptar una nueva etiqueta PPS/UTC.
    bool update();
    bool getTimeAtTicks(uint64_t ticks, int64_t& time_us) override;
    void invalidate() { has_reference_ = false; }

    // Para ajustar el RTC en el PPS SIGUIENTE, sin inventar su fecha.
    // Devuelve la ultima etiqueta ya asociada a un PPS.
    bool lastReference(uint64_t& ticks, int64_t& local_epoch_s) const;

private:
    PeriodicTickCapture& pps_;
    GPS_Module& gps_;
    bool has_reference_ = false;
    uint64_t ref_ticks_ = 0;
    uint32_t ref_seq_ = 0;
    int64_t ref_time_us_ = 0;
};
