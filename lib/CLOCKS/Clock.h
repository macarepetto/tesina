#pragma once

#include <stdint.h>
#include "Pcnt_Extended.h"

// Todos los relojes entregan microsegundos sobre la MISMA escala de fecha/hora.
// Devuelven false cuando no disponen de una referencia válida.
// No consumen muestras PPS ni escriben en el RTC.
class Clock {
public:
    explicit Clock(PcntExtended& counter) : counter_(counter) {}
    virtual ~Clock() = default;

    // Consulta el instante actual: una sola lectura del contador compartido.
    virtual bool getTime(int64_t& time_us) {
        return getTimeAtTicks(counter_.read(), time_us);
    }

    // Permite comparar TODOS los relojes en el MISMO instante físico,
    // por ejemplo, en los ticks capturados en un flanco PPS o un evento.
    virtual bool getTimeAtTicks(uint64_t ticks, int64_t& time_us) = 0;

protected:
    static constexpr uint64_t TICKS_PER_SECOND = 32768ULL;
    static constexpr uint64_t MAX_EDGE_AGE_TICKS = 49152ULL; // 1,5 s

    static int64_t ticksToUs(uint64_t ticks) {
        return static_cast<int64_t>(
            (ticks * 1000000ULL + TICKS_PER_SECOND / 2ULL) / TICKS_PER_SECOND
        );
    }

    PcntExtended& counter_;
};
