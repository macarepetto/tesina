#ifndef WIFI_MODULE_H
#define WIFI_MODULE_H

#include <Arduino.h>
#include <WiFi.h>

class WiFi_Module {
public:
    WiFi_Module(const char* ssid,
                         const char* password,
                         const char* serverIp,
                         uint16_t serverPort,
                         const char* deviceId);

    void begin();
    void update();

    bool isWifiConnected() const;
    bool isServerConnected();

    void sendLine(const String& line);

private:
    const char* _ssid;
    const char* _password;
    const char* _serverIp;
    uint16_t _serverPort;
    const char* _deviceId;

    WiFiClient _client;
    unsigned long _lastWifiAttemptMs = 0;
    unsigned long _lastServerAttemptMs = 0;

    void connectWifi();
    void connectServer();
};

#endif