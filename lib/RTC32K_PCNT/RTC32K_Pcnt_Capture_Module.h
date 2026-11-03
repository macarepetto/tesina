#ifndef RTC32K_PCNT_CAPTURE_MODULE_H
#define RTC32K_PCNT_CAPTURE_MODULE_H

#include <Arduino.h>
#include <stdint.h>

// Captura integrada de la salida 32 kHz del DS3231 y del PPS del GNSS.
//
// Mantiene exactamente el esquema actualmente utilizado:
// - PCNT_UNIT_0 cuenta flancos ascendentes del RTC (32.768 kHz).
// - El limite alto del PCNT se acumula en una ISR.
// - La ISR del PPS captura el acumulado de ticks, la secuencia y micros().
// - El loop obtiene una copia atomica y vacia los PPS pendientes.
//
// No usa la implementacion antigua RTC32K_Pcnt_Module.
namespace RTC32K_Pcnt_Capture {

struct PpsSample {
    uint32_t ticks_raw = 0;
    uint32_t seq = 0;
    uint32_t pending = 0;
    uint32_t pps_time_us = 0;
    uint32_t pps_period_us = 0;
};

// Configura el PCNT y registra las dos interrupciones necesarias.
// Ante un fallo de configuracion se conserva el bloqueo diagnostico original.
void begin(int pin_rtc_32k, int pin_gnss_pps);

// Lee el ultimo PPS y consume los eventos pendientes.
// Si llegaron varios PPS antes de la lectura, 'pending' informa cuantos.
bool consumePps(PpsSample& sample);

// Lectura atomica no destructiva usada para diagnosticar el retardo RMC.
void snapshotLastPps(uint32_t& seq, uint32_t& pps_time_us);

// Lee el contador PCNT EN ESTE MOMENTO, incluso sin PPS nuevo.
// No consume PPS pendientes ni modifica el contador ni sus ISR.
// Se usa solamente para informar las horas estimadas durante holdover.
bool snapshotCurrentTicks(uint32_t& ticks_raw);

}  // namespace RTC32K_Pcnt_Capture

#endif
