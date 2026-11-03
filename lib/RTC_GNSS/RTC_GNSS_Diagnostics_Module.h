#ifndef RTC_GNSS_DIAGNOSTICS_MODULE_H
#define RTC_GNSS_DIAGNOSTICS_MODULE_H

#include <Arduino.h>
#include "GPS_Module.h"
#include "RTC_Module.h"
#include "Kalman_Filter.h"
#include "Three_Clocks_Module.h"

// Solo datos observacionales para log. No gobierna la ISR, el RTC ni Kalman.
struct RTC_GNSS_InitState {
    bool utc_reference_ready = false;
    bool rtc_synced_utc = false;
    uint32_t rtc_sync_pps_seq = 0;
    uint32_t rtc_sync_start_delay_us = 0;
    uint32_t statistics_start_pps_seq = 0;
};

class RTC_GNSS_Diagnostics_Module {
public:
    RTC_GNSS_Diagnostics_Module(GPS_Module& gps, RTC_Module& rtc,
                                Three_Clocks_Module& clocks);

    // Procesa solo una revision RMC nueva, leyendo el snapshot PPS existente.
    void observeRmcAfterPps();

    // Conserva los nombres de todos los campos JSON anteriores.
    void buildJsonFields(char* out, size_t len,
                         const RTC_GNSS_InitState& init,
                         bool pps_usable = false, uint32_t seq = 0,
                         uint32_t ticks_raw = 0,
                         const Kalman_State* state = nullptr,
                         bool holdover_estimated = false);

private:
    GPS_Module& gps_;
    RTC_Module& rtc_;
    Three_Clocks_Module& clocks_;
    uint32_t last_rmc_revision_ = 0;
    int32_t last_rmc_after_pps_us_ = -1;
    uint32_t last_rmc_pps_seq_ = 0;

    void appendRmc(char* out, size_t len, uint32_t seq,
                   bool rmc_recent, const DateTime& rmc_local) const;
    void appendUbx(char* out, size_t len) const;
    static void appendInit(char* out, size_t len,
                           const RTC_GNSS_InitState& init);
};

#endif
