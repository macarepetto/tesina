#include "WiFi_Module.h"

WiFi_Module::WiFi_Module(const char* ssid,
                         const char* password,
                         const char* serverIp,
                         uint16_t serverPort,
                         const char* deviceId)
    : _ssid(ssid),
      _password(password),
      _serverIp(serverIp),
      _serverPort(serverPort),
      _deviceId(deviceId) {}

void WiFi_Module::begin() {
    WiFi.mode(WIFI_STA);
    Serial.println("[WiFi] begin()");
    connectWifi();     // intenta y espera un poco
    connectServer();   // intenta TCP si hay WiFi
}

void WiFi_Module::update() {
    // Reintento WiFi
    if (!isWifiConnected()) {
        if (millis() - _lastWifiAttemptMs > 5000) {
            Serial.println("[WiFi] WiFi caido -> reintento");
            connectWifi();
        }
        return; // sin WiFi no tiene sentido TCP
    }

    // Reintento TCP
    if (!isServerConnected()) {
        if (millis() - _lastServerAttemptMs > 3000) {
            Serial.println("[WiFi] TCP caido -> reintento");
            connectServer();
        }
    }
}

bool WiFi_Module::isWifiConnected() const {
    return WiFi.status() == WL_CONNECTED;
}

bool WiFi_Module::isServerConnected() {
    return _client.connected();
}

void WiFi_Module::connectWifi() {
    _lastWifiAttemptMs = millis();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("[WiFi] ya conectado. IP=");
        Serial.println(WiFi.localIP());
        return;
    }

    Serial.print("[WiFi] conectando a SSID: ");
    Serial.println(_ssid);

    WiFi.begin(_ssid, _password);

    // Espera hasta 10 s (si no, sigue y reintenta en update())
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < 10000) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("[WiFi] conectado! IP=");
        Serial.println(WiFi.localIP());
    } else {
        Serial.print("[WiFi] NO conectado. status=");
        Serial.println((int)WiFi.status());
    }
}

void WiFi_Module::connectServer() {
    _lastServerAttemptMs = millis();

    if (!isWifiConnected()) {
        Serial.println("[WiFi] sin WiFi -> no intento TCP");
        return;
    }

    if (_client.connected()) {
        Serial.println("[WiFi] TCP ya conectado");
        return;
    }

    _client.stop();

    Serial.print("[WiFi] conectando TCP a ");
    Serial.print(_serverIp);
    Serial.print(":");
    Serial.println(_serverPort);

    bool ok = _client.connect(_serverIp, _serverPort);
    Serial.println(ok ? "[WiFi] TCP conectado!" : "[WiFi] TCP NO conecta");
}

void WiFi_Module::sendLine(const String& line) {
    if (!isWifiConnected()) return;
    if (!isServerConnected()) return;

    _client.println(line);
}