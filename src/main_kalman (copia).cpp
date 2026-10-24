#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include "driver/pcnt.h"
#include "Kalman_Filter.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// ==================== PINES ====================

#define PIN_32K_RTC 18
#define PIN_PPS_GPS 27

// ==================== GNSS UART / NMEA ====================
// Conectar TX del GNSS al GPIO indicado acá.
// Si tu módulo GNSS usa otro pin, cambiá solamente PIN_GPS_RX.
#define PIN_GPS_RX 16
#define PIN_GPS_TX 17
#define GPS_BAUD 9600

// ==================== PCNT ====================

#define PCNT_UNIT_USED    PCNT_UNIT_0
#define PCNT_CHANNEL_USED PCNT_CHANNEL_0
#define PCNT_HIGH_LIMIT 30000
#define PCNT_LOW_LIMIT -30000
#define PPS_TIME_TOLERANCE_US 200000UL

static constexpr int64_t MAX_DIFF_TICKS = 10;
static constexpr double TICK_US = 1000000.0 / 32768.0;

// ==================== HOLDOVER ====================
// Si pasan más de 1,5 s sin PPS, se considera pérdida de referencia.
static constexpr uint32_t HOLDOVER_TIMEOUT_MS = 1500;

// En holdover se hace una predicción por segundo.
static constexpr uint32_t HOLDOVER_STEP_MS = 1000;

// ==================== AGING DS3231 ====================

// Cambiar solamente esta línea para cada ensayo.
static constexpr int8_t AGING_VALUE = -13;
static constexpr uint8_t DS3231_ADDRESS = 0x68;
static constexpr uint8_t REG_CONTROL = 0x0E;
static constexpr uint8_t REG_STATUS = 0x0F;
static constexpr uint8_t REG_AGING = 0x10;
static constexpr uint8_t BIT_CONV = 5;
static constexpr uint8_t BIT_BSY = 2;

// ==================== WIFI ====================

const char* WIFI_SSID = "PATAN";
const char* WIFI_PASS = "autoslocos";
const char* SERVER_IP = "192.168.0.119";
const uint16_t SERVER_PORT = 8080;
const char* DEVICE_ID = "esp32-prototipo-1";

WiFiClient client;

// ==================== KALMAN ====================

Kalman_Filter kalman;

// ==================== VARIABLES COMPARTIDAS ISR / LOOP ====================

portMUX_TYPE pcnt_mux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t pcnt_base = 0;
volatile uint32_t ticks_raw_isr = 0;
volatile uint32_t pps_seq = 0;
volatile uint32_t pps_pending = 0;
volatile uint32_t pps_time_us_isr = 0;
volatile uint32_t pps_period_us_isr = 0;

int8_t aging_aplicado = 0;

// ==================== I2C / DS3231 ====================

bool rtcWriteRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool rtcReadRegister(uint8_t reg, uint8_t& value) {
    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom(DS3231_ADDRESS, (uint8_t)1) != 1) {
        return false;
    }
    value = Wire.read();
    return true;
}

bool waitRtcNotBusy(uint32_t timeout_ms) {
    const uint32_t inicio = millis();
    while (millis() - inicio < timeout_ms) {
        uint8_t status = 0;
        if (!rtcReadRegister(REG_STATUS, status)) {
            return false;
        }
        if ((status & (1U << BIT_BSY)) == 0) {
            return true;
        }
        delay(5);
    }
    return false;
}

bool forceAgingUpdate() {
    if (!waitRtcNotBusy(500)) {
        return false;
    }
    uint8_t control = 0;
    if (!rtcReadRegister(REG_CONTROL, control)) {
        return false;
    }
    control |= (1U << BIT_CONV);
    if (!rtcWriteRegister(REG_CONTROL, control)) {
        return false;
    }
    // CONV permanece en 1 hasta que el DS3231 termina su actualización interna.
    const uint32_t inicio = millis();
    while (millis() - inicio < 1000) {
        if (!rtcReadRegister(REG_CONTROL, control)) {
            return false;
        }
        if ((control & (1U << BIT_CONV)) == 0) {
            return waitRtcNotBusy(500);
        }
        delay(5);
    }
    return false;
}

bool applyAgingValue(int8_t value) {
    if (!rtcWriteRegister(REG_AGING, (uint8_t)value)) {
        return false;
    }
    if (!forceAgingUpdate()) {
        return false;
    }
    uint8_t readback = 0;
    if (!rtcReadRegister(REG_AGING, readback)) {
        return false;
    }
    aging_aplicado = (int8_t)readback;
    return aging_aplicado == value;
}

// ==================== FECHA/HORA GNSS + RTC ====================

void sendJson(const char* json);

struct UtcDateTime {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    bool valid;
};

static UtcDateTime gnss_datetime = {0, 0, 0, 0, 0, 0, false};
static int64_t gnss_epoch_s = 0;
static bool gnss_datetime_valid = false;
static uint32_t gnss_msg_count = 0;
// Momento local (millis) en que llegó la última sentencia RMC válida.
// Se usa sólo para decidir si hay una hora GNSS fresca contra la cual comparar el RTC.
static uint32_t gnss_last_valid_ms = 0;
// Si la última RMC válida tiene más de este tiempo, no calculamos error RTC-GNSS.
// Así evitamos comparar el RTC contra una hora GNSS vieja.
static constexpr uint32_t GNSS_FRESH_TIMEOUT_MS = 3000;

static bool rtc_sincronizado = false;
static uint32_t pps_seq_sync = 0;
static int64_t ref_epoch_sync_s = 0;

static char nmea_line[140];
static size_t nmea_len = 0;

uint8_t decToBcd(uint8_t value) {
    return ((value / 10) << 4) | (value % 10);
}

uint8_t bcdToDec(uint8_t value) {
    return ((value >> 4) * 10) + (value & 0x0F);
}

int64_t daysFromCivil(int y, unsigned m, unsigned d) {
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy =
        (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe =
        yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

void civilFromDays(int64_t z, int& y, unsigned& m, unsigned& d) {
    z += 719468;
    const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = (unsigned)(z - era * 146097);
    const unsigned yoe =
        (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    y = (int)yoe + (int)era * 400;
    const unsigned doy =
        doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    d = doy - (153 * mp + 2) / 5 + 1;
    m = mp + (mp < 10 ? 3 : -9);
    y += (m <= 2);
}

int64_t dateTimeToEpoch(const UtcDateTime& dt) {
    int64_t days = daysFromCivil(dt.year, dt.month, dt.day);
    return days * 86400LL +
           (int64_t)dt.hour * 3600LL +
           (int64_t)dt.minute * 60LL +
           (int64_t)dt.second;
}

void epochToDateTime(int64_t epoch_s, UtcDateTime& dt) {
    int64_t days = epoch_s / 86400LL;
    int64_t seconds_of_day = epoch_s % 86400LL;
    if (seconds_of_day < 0) {
        seconds_of_day += 86400LL;
        days--;
    }
    int year = 0;
    unsigned month = 0;
    unsigned day = 0;
    civilFromDays(days, year, month, day);
    dt.year = year;
    dt.month = (int)month;
    dt.day = (int)day;
    dt.hour = (int)(seconds_of_day / 3600LL);
    dt.minute = (int)((seconds_of_day % 3600LL) / 60LL);
    dt.second = (int)(seconds_of_day % 60LL);
    dt.valid = true;
}

void epochToIso(int64_t epoch_s, char* out, size_t len) {
    UtcDateTime dt;
    epochToDateTime(epoch_s, dt);
    snprintf(
        out,
        len,
        "%04d-%02d-%02dT%02d:%02d:%02dZ",
        dt.year,
        dt.month,
        dt.day,
        dt.hour,
        dt.minute,
        dt.second
    );
}

uint8_t ds3231DayOfWeekFromEpoch(int64_t epoch_s) {
    int64_t days = epoch_s / 86400LL;
    // 1970-01-01 fue jueves.
    // Usamos 1=domingo, 2=lunes, ..., 7=sábado.
    int dow = (int)((days + 4) % 7);
    if (dow < 0) {
        dow += 7;
    }
    return (uint8_t)(dow + 1);
}

bool setRTCFromEpoch(int64_t epoch_s) {
    UtcDateTime dt;
    epochToDateTime(epoch_s, dt);
    if (dt.year < 2000 || dt.year > 2099) {
        return false;
    }
    uint8_t year_2_digits = (uint8_t)(dt.year - 2000);
    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write((uint8_t)0x00);                 // Registro segundos
    Wire.write(decToBcd((uint8_t)dt.second));
    Wire.write(decToBcd((uint8_t)dt.minute));
    Wire.write(decToBcd((uint8_t)dt.hour));    // 24 h
    Wire.write(decToBcd(ds3231DayOfWeekFromEpoch(epoch_s)));
    Wire.write(decToBcd((uint8_t)dt.day));
    Wire.write(decToBcd((uint8_t)dt.month));
    Wire.write(decToBcd(year_2_digits));
    return Wire.endTransmission() == 0;
}

bool readRTCToEpoch(int64_t& epoch_s) {
    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write((uint8_t)0x00);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom(DS3231_ADDRESS, (uint8_t)7) != 7) {
        return false;
    }
    uint8_t raw_sec = Wire.read();
    uint8_t raw_min = Wire.read();
    uint8_t raw_hour = Wire.read();
    Wire.read(); // día de semana; no se usa para el cálculo
    uint8_t raw_day = Wire.read();
    uint8_t raw_month = Wire.read();
    uint8_t raw_year = Wire.read();
    UtcDateTime dt;
    dt.second = bcdToDec(raw_sec & 0x7F);
    dt.minute = bcdToDec(raw_min & 0x7F);
    dt.hour = bcdToDec(raw_hour & 0x3F);       // modo 24 h
    dt.day = bcdToDec(raw_day & 0x3F);
    dt.month = bcdToDec(raw_month & 0x1F);
    dt.year = 2000 + bcdToDec(raw_year);
    dt.valid = true;
    if (
        dt.year < 2020 || dt.year > 2099 ||
        dt.month < 1 || dt.month > 12 ||
        dt.day < 1 || dt.day > 31 ||
        dt.hour < 0 || dt.hour > 23 ||
        dt.minute < 0 || dt.minute > 59 ||
        dt.second < 0 || dt.second > 59
    ) {
        return false;
    }
    epoch_s = dateTimeToEpoch(dt);
    return true;
}

bool nmeaChecksumOk(const char* line) {
    if (line == nullptr || line[0] != '$') {
        return false;
    }
    const char* star = strchr(line, '*');
    if (star == nullptr || strlen(star) < 3) {
        return false;
    }
    uint8_t calc = 0;
    for (const char* p = line + 1; p < star; p++) {
        calc ^= (uint8_t)(*p);
    }
    char checksum_text[3];
    checksum_text[0] = star[1];
    checksum_text[1] = star[2];
    checksum_text[2] = '\0';
    uint8_t received = (uint8_t)strtol(checksum_text, nullptr, 16);
    return calc == received;
}

int twoDigits(const char* p) {
    if (!isdigit((unsigned char)p[0]) || !isdigit((unsigned char)p[1])) {
        return -1;
    }
    return (p[0] - '0') * 10 + (p[1] - '0');
}

bool parseRMC(const char* line, UtcDateTime& dt, int64_t& epoch_s) {
    if (!nmeaChecksumOk(line)) {
        return false;
    }
    if (
        strncmp(line, "$GPRMC", 6) != 0 &&
        strncmp(line, "$GNRMC", 6) != 0
    ) {
        return false;
    }
    char work[140];
    strncpy(work, line, sizeof(work));
    work[sizeof(work) - 1] = '\0';
    char* star = strchr(work, '*');
    if (star != nullptr) {
        *star = '\0';
    }
    char* fields[16];
    int n = 0;
    char* token = strtok(work, ",");
    while (token != nullptr && n < 16) {
        fields[n++] = token;
        token = strtok(nullptr, ",");
    }
    // RMC:
    // 0 tipo
    // 1 hora UTC hhmmss.sss
    // 2 estado A/V
    // 9 fecha ddmmyy
    if (n < 10) {
        return false;
    }
    const char* time_text = fields[1];
    const char* status_text = fields[2];
    const char* date_text = fields[9];
    if (status_text[0] != 'A') {
        return false;
    }
    if (strlen(time_text) < 6 || strlen(date_text) < 6) {
        return false;
    }
    int hh = twoDigits(time_text + 0);
    int mm = twoDigits(time_text + 2);
    int ss = twoDigits(time_text + 4);
    int day = twoDigits(date_text + 0);
    int month = twoDigits(date_text + 2);
    int yy = twoDigits(date_text + 4);
    if (
        hh < 0 || mm < 0 || ss < 0 ||
        day < 0 || month < 0 || yy < 0
    ) {
        return false;
    }
    dt.year = 2000 + yy;
    dt.month = month;
    dt.day = day;
    dt.hour = hh;
    dt.minute = mm;
    dt.second = ss;
    dt.valid = true;
    epoch_s = dateTimeToEpoch(dt);
    return true;
}

void procesarGNSS() {
    while (Serial2.available() > 0) {
        char c = (char)Serial2.read();
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            nmea_line[nmea_len] = '\0';
            UtcDateTime dt;
            int64_t epoch = 0;
            if (parseRMC(nmea_line, dt, epoch)) {
                gnss_datetime = dt;
                gnss_epoch_s = epoch;
                gnss_datetime_valid = true;
                gnss_msg_count++;
                gnss_last_valid_ms = millis();
            }
            nmea_len = 0;
        } else {
            if (nmea_len < sizeof(nmea_line) - 1) {
                nmea_line[nmea_len++] = c;
            } else {
                nmea_len = 0;
            }
        }
    }
}

void intentarSincronizarRtcConGnss(bool interval_valid, uint32_t current_pps_seq) {
    if (rtc_sincronizado) {
        return;
    }
    if (!interval_valid) {
        return;
    }
    if (!gnss_datetime_valid) {
        return;
    }
    if (setRTCFromEpoch(gnss_epoch_s)) {
        rtc_sincronizado = true;
        pps_seq_sync = current_pps_seq;
        ref_epoch_sync_s = gnss_epoch_s;
        char iso[32];
        epochToIso(ref_epoch_sync_s, iso, sizeof(iso));
        Serial.print("RTC sincronizado con GNSS en ");
        Serial.print(iso);
        Serial.print(" | pps_seq_sync=");
        Serial.println(pps_seq_sync);
    } else {
        Serial.println("ERROR: no se pudo sincronizar el RTC con GNSS");
    }
}

void quotedIsoOrNull(bool valid, int64_t epoch_s, char* out, size_t len) {
    if (!valid) {
        snprintf(out, len, "null");
        return;
    }
    char iso[32];
    epochToIso(epoch_s, iso, sizeof(iso));
    snprintf(out, len, "\"%s\"", iso);
}

void sendRtcCheckJson(uint32_t current_pps_seq, uint32_t now_ms) {
    char gnss_last_json[40];
    char gnss_json[40];
    char rtc_json[40];
    char rtc_error_json[24];
    char sync_seq_json[24];
    char gnss_age_json[24];
    // gnss_last_utc: última hora GNSS válida recibida, aunque ya sea vieja.
    quotedIsoOrNull(
        gnss_datetime_valid,
        gnss_epoch_s,
        gnss_last_json,
        sizeof(gnss_last_json)
    );
    snprintf(gnss_json, sizeof(gnss_json), "null");
    snprintf(rtc_json, sizeof(rtc_json), "null");
    snprintf(rtc_error_json, sizeof(rtc_error_json), "null");
    snprintf(sync_seq_json, sizeof(sync_seq_json), "null");
    snprintf(gnss_age_json, sizeof(gnss_age_json), "null");
    // Leemos SIEMPRE la hora calendario del RTC, aunque todavía no se haya
    // sincronizado con GNSS. Así queda en el log qué hora cree tener el RTC.
    int64_t rtc_epoch_s = 0;
    bool rtc_read_ok = readRTCToEpoch(rtc_epoch_s);
    if (rtc_read_ok) {
        quotedIsoOrNull(
            true,
            rtc_epoch_s,
            rtc_json,
            sizeof(rtc_json)
        );
    }
    if (rtc_sincronizado) {
        snprintf(
            sync_seq_json,
            sizeof(sync_seq_json),
            "%lu",
            (unsigned long)pps_seq_sync
        );
    }
    // Comparación calendario SIMPLE:
    //   error = RTC - GNSS
    // Sólo se calcula si hay una RMC válida reciente. No usamos GNSS estimado,
    // ni referencia calculada con pps_seq, ni nada que pueda quedar congelado
    // cuando falta PPS.
    bool gnss_fresh = false;
    uint32_t gnss_age_ms = 0;
    if (gnss_datetime_valid) {
        gnss_age_ms = now_ms - gnss_last_valid_ms;
        snprintf(
            gnss_age_json,
            sizeof(gnss_age_json),
            "%lu",
            (unsigned long)gnss_age_ms
        );
        gnss_fresh = (gnss_age_ms <= GNSS_FRESH_TIMEOUT_MS);
    }
    if (gnss_fresh) {
        quotedIsoOrNull(
            true,
            gnss_epoch_s,
            gnss_json,
            sizeof(gnss_json)
        );
        if (rtc_read_ok) {
            int64_t rtc_error_s = rtc_epoch_s - gnss_epoch_s;
            snprintf(
                rtc_error_json,
                sizeof(rtc_error_json),
                "%lld",
                (long long)rtc_error_s
            );
        }
    }
    char json[1000];
    snprintf(
        json,
        sizeof(json),
        "{\"id\":\"%s\","
        "\"aging\":%d,"
        "\"ms\":%lu,"
        "\"pps_seq\":%lu,"
        "\"modo\":\"RTC_CHECK\","
        "\"rtc_sincronizado\":%s,"
        "\"rtc_read_ok\":%s,"
        "\"gnss_datetime_valid\":%s,"
        "\"gnss_fresh\":%s,"
        "\"gnss_msg_count\":%lu,"
        "\"gnss_age_ms\":%s,"
        "\"gnss_last_utc\":%s,"
        "\"gnss_utc\":%s,"
        "\"rtc_utc\":%s,"
        "\"rtc_error_s\":%s,"
        "\"rtc_error_valid\":%s,"
        "\"rtc_sync_pps_seq\":%s}",
        DEVICE_ID,
        (int)aging_aplicado,
        (unsigned long)now_ms,
        (unsigned long)current_pps_seq,
        rtc_sincronizado ? "true" : "false",
        rtc_read_ok ? "true" : "false",
        gnss_datetime_valid ? "true" : "false",
        gnss_fresh ? "true" : "false",
        (unsigned long)gnss_msg_count,
        gnss_age_json,
        gnss_last_json,
        gnss_json,
        rtc_json,
        rtc_error_json,
        (gnss_fresh && rtc_read_ok) ? "true" : "false",
        sync_seq_json
    );
    sendJson(json);
}

// ==================== INTERRUPCIONES ====================

void IRAM_ATTR onPCNTLimit(void* arg) {
    uint32_t status = 0;
    pcnt_get_event_status(PCNT_UNIT_USED, &status);
    if (status & PCNT_EVT_H_LIM) {
        portENTER_CRITICAL_ISR(&pcnt_mux);
        pcnt_base += PCNT_HIGH_LIMIT;
        portEXIT_CRITICAL_ISR(&pcnt_mux);
    }
}

void IRAM_ATTR onPPS() {
    const uint32_t now_us = micros();
    int16_t hw_count = 0;
    portENTER_CRITICAL_ISR(&pcnt_mux);
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

// ==================== CONFIGURACIÓN ====================

void setupPCNT() {
    pinMode(PIN_32K_RTC, INPUT_PULLUP);
    pcnt_config_t config = {};
    config.pulse_gpio_num = PIN_32K_RTC;
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
}

void connectWiFi() {
    Serial.printf("Conectando a WiFi %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("WiFi conectado. IP: ");
    Serial.println(WiFi.localIP());
}

void maintainServerConnection() {
    static uint32_t last_attempt_ms = 0;
    if (client.connected()) {
        return;
    }
    if (millis() - last_attempt_ms < 2000) {
        return;
    }
    last_attempt_ms = millis();
    Serial.println("Intentando conectar al servidor...");
    client.stop();
    if (client.connect(SERVER_IP, SERVER_PORT)) {
        client.setNoDelay(true);
        Serial.println("Conectado al servidor");
    } else {
        Serial.println("Servidor no disponible; se reintentará");
    }
}

void sendJson(const char* json) {
    Serial.println(json);
    if (client.connected()) {
        client.println(json);
    }
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Wire.begin();
    Serial2.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    Serial.println("GNSS NMEA iniciado por Serial2");
    Serial.print("GPS RX GPIO: ");
    Serial.println(PIN_GPS_RX);
    uint8_t aging_anterior_raw = 0;
    if (!rtcReadRegister(REG_AGING, aging_anterior_raw)) {
        Serial.println("ERROR: no se detectó el DS3231 por I2C");
        while (true) {
            delay(1000);
        }
    }
    Serial.printf(
        "Aging anterior: %d | Aging solicitado: %d\n",
        (int)(int8_t)aging_anterior_raw,
        (int)AGING_VALUE
    );
    if (!applyAgingValue(AGING_VALUE)) {
        Serial.println("ERROR: no se pudo aplicar/verificar el aging");
        while (true) {
            delay(1000);
        }
    }
    Serial.printf("Aging aplicado y verificado: %d\n", (int)aging_aplicado);
    setupPCNT();
    pinMode(PIN_PPS_GPS, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_PPS_GPS), onPPS, RISING);
    connectWiFi();
    Serial.println("PCNT iniciado. Esperando PPS...");
}

// ==================== MEDICIÓN ====================

void loop() {
    procesarGNSS();
    maintainServerConnection();
    static bool hay_anterior = false;
    static uint32_t ticks_raw_prev = 0;
    static uint32_t seq_prev = 0;
    static uint32_t pps_time_us_prev = 0;
    static int64_t offset_ticks = 0;
    static uint32_t last_pps_seen_ms = 0;
    static uint32_t last_holdover_predict_ms = 0;
    static uint32_t holdover_steps = 0;
    static bool en_holdover = false;
    uint32_t ticks_raw = 0;
    uint32_t seq = 0;
    uint32_t pending = 0;
    uint32_t pps_time_us = 0;
    uint32_t pps_period_us = 0;
    bool hay_muestra = false;
    portENTER_CRITICAL(&pcnt_mux);
    if (pps_pending > 0) {
        ticks_raw = ticks_raw_isr;
        seq = pps_seq;
        pending = pps_pending;
        pps_time_us = pps_time_us_isr;
        pps_period_us = pps_period_us_isr;
        pps_pending = 0;
        hay_muestra = true;
    }
    portEXIT_CRITICAL(&pcnt_mux);
    // ============================================================
    // CASO 1: NO LLEGÓ PPS NUEVO
    // ============================================================
    if (!hay_muestra) {
        if (!hay_anterior) {
            return;
        }
        const uint32_t now_ms = millis();
        const uint32_t ms_since_last_pps = now_ms - last_pps_seen_ms;
        const bool pps_perdido =
            ms_since_last_pps > HOLDOVER_TIMEOUT_MS;
        const bool corresponde_predecir =
            now_ms - last_holdover_predict_ms >= HOLDOVER_STEP_MS;
        if (pps_perdido && corresponde_predecir) {
            en_holdover = true;
            holdover_steps++;
            // En holdover NO hay medición nueva.
            // Solo se aplica la etapa de predicción:
            //
            //     phi(k+1) = phi(k) + f(k)
            //     f(k+1)   = f(k)
            kalman.predict();
            Kalman_State kalman_state = kalman.getState();
            const double offset_us =
                (double)offset_ticks * TICK_US;
            const double kalman_phi_us =
                kalman_state.phi * TICK_US;
            char json[1400];
            snprintf(
                json,
                sizeof(json),
                "{\"id\":\"%s\","
                "\"aging\":%d,"
                "\"ms\":%lu,"
                "\"modo\":\"HOLDOVER\","
                "\"first\":false,"
                "\"pps_seq\":%lu,"
                "\"pps_pending\":0,"
                "\"pps_period_us\":0,"
                "\"ms_since_last_pps\":%lu,"
                "\"holdover_steps\":%lu,"
                "\"offset_medido_disponible\":false,"
                "\"kalman_prediccion\":true,"
                "\"kalman_actualizado\":false,"
                "\"ticks\":null,"
                "\"ticks_window\":null,"
                "\"ticks_raw\":null,"
                "\"seq_delta\":0,"
                "\"elapsed_us\":null,"
                "\"interval_valid\":false,"
                "\"diff_outlier\":false,"
                "\"missed_pps\":0,"
                "\"diff\":null,"
                "\"offset_ticks\":%lld,"
                "\"offset_us\":%.3f,"
                "\"kalman_medicion\":\"none\","
                "\"kalman_z_phi\":null,"
                "\"kalman_phi\":%.6f,"
                "\"kalman_phi_us\":%.3f,"
                "\"kalman_f\":%.6f,"
                "\"kalman_p00\":%.6f,"
                "\"kalman_p01\":%.6f,"
                "\"kalman_p10\":%.6f,"
                "\"kalman_p11\":%.6f,"
                "\"kalman_y_phi\":%.6f,"
                "\"kalman_k_phi\":%.6f,"
                "\"kalman_k_f\":%.6f,"
                "\"residual_us\":null}",
                DEVICE_ID,
                (int)aging_aplicado,
                (unsigned long)now_ms,
                (unsigned long)seq_prev,
                (unsigned long)ms_since_last_pps,
                (unsigned long)holdover_steps,
                (long long)offset_ticks,
                offset_us,
                kalman_state.phi,
                kalman_phi_us,
                kalman_state.f,
                kalman_state.p00,
                kalman_state.p01,
                kalman_state.p10,
                kalman_state.p11,
                kalman_state.y0,
                kalman_state.k00,
                kalman_state.k10
            );
            sendJson(json);
            sendRtcCheckJson(seq_prev, now_ms);
            last_holdover_predict_ms = now_ms;
        }
        return;
    }
    // ============================================================
    // CASO 2: LLEGÓ PPS NUEVO
    // ============================================================
    const uint32_t now_ms = millis();
    const bool venia_de_holdover = en_holdover;
    const uint32_t holdover_steps_previos = holdover_steps;
    last_pps_seen_ms = now_ms;
    last_holdover_predict_ms = now_ms;
    if (!hay_anterior) {
        ticks_raw_prev = ticks_raw;
        seq_prev = seq;
        pps_time_us_prev = pps_time_us;
        hay_anterior = true;
        last_pps_seen_ms = now_ms;
        last_holdover_predict_ms = now_ms;
        char json[700];
        snprintf(
            json,
            sizeof(json),
            "{\"id\":\"%s\","
            "\"aging\":%d,"
            "\"ms\":%lu,"
            "\"pps_seq\":%lu,"
            "\"pps_pending\":%lu,"
            "\"pps_period_us\":%lu,"
            "\"modo\":\"PPS_INICIAL\","
            "\"first\":true,"
            "\"offset_medido_disponible\":false,"
            "\"kalman_prediccion\":false,"
            "\"kalman_actualizado\":false,"
            "\"ticks\":0,"
            "\"ticks_window\":0,"
            "\"ticks_raw\":%lu,"
            "\"seq_delta\":0,"
            "\"elapsed_us\":0,"
            "\"interval_valid\":false,"
            "\"diff_outlier\":false,"
            "\"missed_pps\":0,"
            "\"diff\":0,"
            "\"offset_ticks\":0,"
            "\"offset_us\":0.000,"
            "\"kalman_phi_us\":0.000,"
            "\"residual_us\":null}",
            DEVICE_ID,
            (int)aging_aplicado,
            (unsigned long)now_ms,
            (unsigned long)seq,
            (unsigned long)pending,
            (unsigned long)pps_period_us,
            (unsigned long)ticks_raw
        );
        sendJson(json);
        sendRtcCheckJson(seq, now_ms);
        return;
    }
    const uint32_t ticks_window = ticks_raw - ticks_raw_prev;
    const uint32_t seq_delta = seq - seq_prev;
    const uint32_t elapsed_us = pps_time_us - pps_time_us_prev;
    const uint64_t expected_us =
        (uint64_t)seq_delta * 1000000ULL;
    const uint64_t timing_error_us =
        (elapsed_us >= expected_us)
            ? (uint64_t)elapsed_us - expected_us
            : expected_us - (uint64_t)elapsed_us;
    const bool interval_valid =
        seq_delta > 0 &&
        timing_error_us <= PPS_TIME_TOLERANCE_US;
    intentarSincronizarRtcConGnss(interval_valid, seq);
    uint32_t ticks = 0;
    int64_t diff = 0;
    bool diff_outlier = false;
    /*
        Si veníamos de holdover, esta primera muestra se usa
        solamente para reenganchar la referencia.
        No se usa para actualizar offset ni Kalman, porque el intervalo
        entre el último PPS antes de la pérdida y este nuevo PPS no
        representa una ventana normal de 1 segundo.
    */
    if (interval_valid && !venia_de_holdover) {
        ticks = ticks_window / seq_delta;
        const int64_t expected_ticks =
            (int64_t)32768 * (int64_t)seq_delta;
        // Convención:
        // diff = ticks_RTC - ticks_esperados_GNSS
        diff = (int64_t)ticks_window - expected_ticks;
        diff_outlier =
            (diff > MAX_DIFF_TICKS) ||
            (diff < -MAX_DIFF_TICKS);
        if (!diff_outlier) {
            offset_ticks += diff;
        }
    }
    bool kalman_actualizado = false;
    bool kalman_prediccion = false;
    Kalman_State kalman_state;
    if (
        interval_valid &&
        seq_delta == 1 &&
        !diff_outlier &&
        !venia_de_holdover
    ) {
        // Etapa de predicción
        kalman.predict();
        kalman_prediccion = true;
        // Etapa de actualización con medición escalar:
        // z = phi
        kalman.updatePhi((double)offset_ticks);
        kalman_state = kalman.getState();
        kalman_actualizado = true;
    } else {
        kalman_state = kalman.getState();
        kalman_actualizado = false;
        kalman_prediccion = false;
    }
    const uint32_t elapsed_seconds_rounded =
        (elapsed_us + 500000UL) / 1000000UL;
    const uint32_t missed_pps =
        (elapsed_seconds_rounded > seq_delta)
            ? elapsed_seconds_rounded - seq_delta
            : 0;
    const bool offset_medido_disponible =
        interval_valid &&
        !diff_outlier &&
        !venia_de_holdover;
    const double offset_us =
        (double)offset_ticks * TICK_US;
    const double kalman_phi_us =
        kalman_state.phi * TICK_US;
    const double residual_us =
        offset_us - kalman_phi_us;
    char residual_json[40];
    if (offset_medido_disponible) {
        snprintf(
            residual_json,
            sizeof(residual_json),
            "%.3f",
            residual_us
        );
    } else {
        snprintf(
            residual_json,
            sizeof(residual_json),
            "null"
        );
    }
    const char* modo = "PPS_SIN_UPDATE_KALMAN";
    if (venia_de_holdover) {
        modo = "PPS_REENGANCHE";
    } else if (kalman_actualizado) {
        modo = "PPS_KALMAN_Z_PHI";
    }
    char json[1600];
    snprintf(
        json,
        sizeof(json),
        "{\"id\":\"%s\","
        "\"aging\":%d,"
        "\"ms\":%lu,"
        "\"pps_seq\":%lu,"
        "\"pps_pending\":%lu,"
        "\"pps_period_us\":%lu,"
        "\"modo\":\"%s\","
        "\"first\":false,"
        "\"ms_since_last_pps\":0,"
        "\"holdover_steps\":%lu,"
        "\"offset_medido_disponible\":%s,"
        "\"kalman_prediccion\":%s,"
        "\"ticks\":%lu,"
        "\"ticks_window\":%lu,"
        "\"ticks_raw\":%lu,"
        "\"seq_delta\":%lu,"
        "\"elapsed_us\":%lu,"
        "\"interval_valid\":%s,"
        "\"diff_outlier\":%s,"
        "\"missed_pps\":%lu,"
        "\"diff\":%lld,"
        "\"offset_ticks\":%lld,"
        "\"offset_us\":%.3f,"
        "\"kalman_actualizado\":%s,"
        "\"kalman_medicion\":\"phi\","
        "\"kalman_z_phi\":%.6f,"
        "\"kalman_phi\":%.6f,"
        "\"kalman_phi_us\":%.3f,"
        "\"kalman_f\":%.6f,"
        "\"kalman_p00\":%.6f,"
        "\"kalman_p01\":%.6f,"
        "\"kalman_p10\":%.6f,"
        "\"kalman_p11\":%.6f,"
        "\"kalman_y_phi\":%.6f,"
        "\"kalman_k_phi\":%.6f,"
        "\"kalman_k_f\":%.6f,"
        "\"residual_us\":%s}",
        DEVICE_ID,
        (int)aging_aplicado,
        (unsigned long)now_ms,
        (unsigned long)seq,
        (unsigned long)pending,
        (unsigned long)pps_period_us,
        modo,
        (unsigned long)(venia_de_holdover ? holdover_steps_previos : 0),
        offset_medido_disponible ? "true" : "false",
        kalman_prediccion ? "true" : "false",
        (unsigned long)ticks,
        (unsigned long)ticks_window,
        (unsigned long)ticks_raw,
        (unsigned long)seq_delta,
        (unsigned long)elapsed_us,
        interval_valid ? "true" : "false",
        diff_outlier ? "true" : "false",
        (unsigned long)missed_pps,
        (long long)diff,
        (long long)offset_ticks,
        offset_us,
        kalman_actualizado ? "true" : "false",
        (double)offset_ticks,
        kalman_state.phi,
        kalman_phi_us,
        kalman_state.f,
        kalman_state.p00,
        kalman_state.p01,
        kalman_state.p10,
        kalman_state.p11,
        kalman_state.y0,
        kalman_state.k00,
        kalman_state.k10,
        residual_json
    );
    sendJson(json);
    sendRtcCheckJson(seq, now_ms);
    ticks_raw_prev = ticks_raw;
    seq_prev = seq;
    pps_time_us_prev = pps_time_us;
    if (venia_de_holdover) {
        en_holdover = false;
        holdover_steps = 0;
    }
}