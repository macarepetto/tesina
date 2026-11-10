#ifndef RTC_SQW_CLOCK_MODULE_H
#define RTC_SQW_CLOCK_MODULE_H

#include <Arduino.h>
#include <stdint.h>

class RTC_Module;
class Telemetry_Queue_Module;

// Reloj del CALENDARIO del DS3231, independiente de los segundos GNSS.
//
// El flanco descendente del SQW a 1 Hz marca el origen del segundo RTC.
// En ESA ISR se toma el contador PCNT de la salida 32.768 kHz del RTC.
// Fuera de la ISR se asocia el segundo del calendario via I2C.
//
// No modifica la ISR del GNSS PPS, no reinicia PCNT, ni modifica Kalman.
namespace RTC_SQW_Clock {

// Se llama DESPUES de rtc.begin() y RTC32K_Pcnt_Capture::begin().
// Configura la salida INT/SQW del DS3231 a 1 Hz y conecta GPIO19.
// Si falla, el resto del prototipo puede continuar trabajando como antes.
bool begin(int pin_rtc_sqw);

// Avisar DESPUES de cualquier escritura de la hora del calendario RTC.
// La escritura del registro de segundos reinicia su divisor interno;
// por eso se descarta el anclaje viejo hasta un nuevo flanco SQW.
void invalidateOnCalendarWrite();

// Servicio en el loop, NUNCA dentro de una ISR.
// Asocia el flanco SQW a la hora propia del DS3231.
// Si telemetry es nullptr, no publica registros JSON adicionales.
void service(RTC_Module& rtc, const char* device_id = nullptr,
             Telemetry_Queue_Module* telemetry = nullptr);
// Conserva compatibilidad con el servicio anterior, que si publicaba eventos SQW.
void service(RTC_Module& rtc, const char* device_id,
             Telemetry_Queue_Module& telemetry);

// Convierte un instante PCNT a hora propia del RTC con microsegundos.
// Los microsegundos son formato de representacion; la resolucion de
// conteo es de ~30.52 us. Disponible aun sin GNSS, si el calendario RTC
// es valido y ya se registro al menos un flanco SQW.
bool timeAtTicks(uint32_t event_ticks_raw, int64_t& epoch_us,
                 uint32_t& ticks_since_sqw);

// Agrega SOLO el cuarto reloj al JSON de PPS/HOLDOVER ya existente.
// No entrega valores a Kalman ni al reloj RTC/PPS.
void appendClockJson(char* out, size_t len, bool ticks_disponibles,
                     uint32_t ticks_raw);

// Publica la hora propia del RTC exactamente en el PPS GNSS, sin utilizar
// la hora GNSS para calcularla. Es una medicion comparativa independiente.
// No modifica el calculo de offset/skew ni las muestras PPS originales.
void publishAtPps(const char* device_id, uint32_t pps_seq,
                  uint32_t pps_ticks_raw,
                  Telemetry_Queue_Module& telemetry);

}  // namespace RTC_SQW_Clock

#endif
