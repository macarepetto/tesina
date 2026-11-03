#include "RTC32K_Pcnt_Capture_Module.h"
#include "driver/pcnt.h"

namespace RTC32K_Pcnt_Capture {

static constexpr pcnt_unit_t PCNT_UNIT_USED = PCNT_UNIT_0;
static constexpr pcnt_channel_t PCNT_CHANNEL_USED = PCNT_CHANNEL_0;
static constexpr int16_t PCNT_HIGH_LIMIT = 30000;
static constexpr int16_t PCNT_LOW_LIMIT = -30000;
// Un 1PPS genuino llega aproximadamente cada segundo, pero no exactamente.
// No rechazamos por desviarse de 1 s: solo evitamos que una rafaga de flancos
// rapidos se convierta en segundos ficticios para el comparador.
// Esto no prueba por si solo la validez GNSS del flanco aceptado.
static constexpr uint32_t PPS_MIN_GAP_US = 500000UL;

static portMUX_TYPE pcnt_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t pcnt_base = 0;
static volatile uint32_t ticks_raw_isr = 0;
static volatile uint32_t pps_seq = 0;
static volatile uint32_t pps_pending = 0;
static volatile uint32_t pps_time_us_isr = 0;
static volatile uint32_t pps_period_us_isr = 0;

static void IRAM_ATTR onPCNTLimit(void* arg) {
    uint32_t status = 0;
    pcnt_get_event_status(PCNT_UNIT_USED, &status);
    if (status & PCNT_EVT_H_LIM) {
        portENTER_CRITICAL_ISR(&pcnt_mux);
        pcnt_base += PCNT_HIGH_LIMIT;
        portEXIT_CRITICAL_ISR(&pcnt_mux);
    }
}

static void IRAM_ATTR onPPS() {
    const uint32_t now_us = micros();
    portENTER_CRITICAL_ISR(&pcnt_mux);

    // El contador de secuencia tiene que representar pulsos ACEPTADOS.
    // Comparamos con el ultimo aceptado, no con un flanco rechazado:
    // una rafaga no debe mover la referencia temporal ni el PCNT capturado.
    // La resta unsigned tambien funciona al desbordar micros().
    if (pps_seq != 0 &&
        (uint32_t)(now_us - pps_time_us_isr) < PPS_MIN_GAP_US) {
        portEXIT_CRITICAL_ISR(&pcnt_mux);
        return;
    }

    int16_t hw_count = 0;
    pcnt_get_counter_value(PCNT_UNIT_USED, &hw_count);
    if (pps_time_us_isr != 0) {
        pps_period_us_isr = now_us - pps_time_us_isr;
    } else {
        pps_period_us_isr = 0;
    }
    pps_time_us_isr = now_us;
    ticks_raw_isr = pcnt_base + (uint32_t)hw_count;
    pps_seq++;
    pps_pending++;
    portEXIT_CRITICAL_ISR(&pcnt_mux);
}

void begin(int pin_rtc_32k, int pin_gnss_pps) {
    pinMode(pin_rtc_32k, INPUT_PULLUP);
    pcnt_config_t config = {};
    config.pulse_gpio_num = pin_rtc_32k;
    config.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    config.pos_mode = PCNT_COUNT_INC;
    config.neg_mode = PCNT_COUNT_DIS;
    config.lctrl_mode = PCNT_MODE_KEEP;
    config.hctrl_mode = PCNT_MODE_KEEP;
    config.counter_h_lim = PCNT_HIGH_LIMIT;
    config.counter_l_lim = PCNT_LOW_LIMIT;
    config.unit = PCNT_UNIT_USED;
    config.channel = PCNT_CHANNEL_USED;
    if (pcnt_unit_config(&config) != ESP_OK) {
        Serial.println("ERROR: no se pudo configurar PCNT");
        while (true) {
            delay(1000);
        }
    }
    pcnt_filter_disable(PCNT_UNIT_USED);
    pcnt_counter_pause(PCNT_UNIT_USED);
    pcnt_counter_clear(PCNT_UNIT_USED);
    pcnt_base = 0;
    pcnt_event_enable(PCNT_UNIT_USED, PCNT_EVT_H_LIM);
    esp_err_t result = pcnt_isr_service_install(0);
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        Serial.println("ERROR: no se pudo instalar la ISR de PCNT");
        while (true) {
            delay(1000);
        }
    }
    if (pcnt_isr_handler_add(
            PCNT_UNIT_USED,
            onPCNTLimit,
            nullptr
        ) != ESP_OK) {
        Serial.println("ERROR: no se pudo registrar la ISR de PCNT");
        while (true) {
            delay(1000);
        }
    }
    pcnt_counter_resume(PCNT_UNIT_USED);

    // El receptor GPS se inicializa con beginNmeaOnly(): esta es la
    // unica ISR que se registra para su PPS.
    pinMode(pin_gnss_pps, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin_gnss_pps), onPPS, RISING);
}

bool consumePps(PpsSample& sample) {
    bool hay_muestra = false;
    portENTER_CRITICAL(&pcnt_mux);
    if (pps_pending > 0) {
        sample.ticks_raw = ticks_raw_isr;
        sample.seq = pps_seq;
        sample.pending = pps_pending;
        sample.pps_time_us = pps_time_us_isr;
        sample.pps_period_us = pps_period_us_isr;
        pps_pending = 0;
        hay_muestra = true;
    }
    portEXIT_CRITICAL(&pcnt_mux);
    return hay_muestra;
}

void snapshotLastPps(uint32_t& seq, uint32_t& pps_time_us) {
    portENTER_CRITICAL(&pcnt_mux);
    seq = pps_seq;
    pps_time_us = pps_time_us_isr;
    portEXIT_CRITICAL(&pcnt_mux);
}

bool snapshotCurrentTicks(uint32_t& ticks_raw) {
    // Copiar la base en secciones criticas muy cortas. La lectura del
    // hardware queda FUERA de ellas para no demorar la ISR del PPS.
    uint32_t base_before = 0;
    portENTER_CRITICAL(&pcnt_mux);
    base_before = pcnt_base;
    portEXIT_CRITICAL(&pcnt_mux);

    int16_t hw_count = 0;
    const esp_err_t result = pcnt_get_counter_value(PCNT_UNIT_USED, &hw_count);

    uint32_t base_after = 0;
    uint32_t last_pps_ticks = 0;
    portENTER_CRITICAL(&pcnt_mux);
    base_after = pcnt_base;
    last_pps_ticks = ticks_raw_isr;
    portEXIT_CRITICAL(&pcnt_mux);

    // Evita un resultado incongruente si cruzamos 30000 ticks durante
    // la lectura, o si acaba de llegar otro PPS mientras copiamos.
    if (result != ESP_OK || base_before != base_after || hw_count < 0) {
        return false;
    }
    const uint32_t current_ticks = base_before + (uint32_t)hw_count;
    if ((int32_t)(current_ticks - last_pps_ticks) < 0) {
        return false;
    }
    ticks_raw = current_ticks;
    return true;
}

}  // namespace RTC32K_Pcnt_Capture
