#ifndef PPS_CAPTURE_H
#define PPS_CAPTURE_H

#include <Arduino.h>
#include <stdint.h>

// Adaptador de compatibilidad con main: maneja la captura PPS,
// mientras PcntExtended administra hardware y desbordes PCNT.
namespace PPS_Capture {

struct PpsSample {
    uint32_t ticks_raw = 0;
    uint32_t seq = 0;
    uint32_t pending = 0;
    uint32_t pps_time_us = 0;
    uint32_t pps_period_us = 0;
};

// Inicializa PCNT extendido y luego la captura PPS.
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

}

#endif