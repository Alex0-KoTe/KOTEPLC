#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include <String>

// Простой менеджер Wi-Fi: подключение к точке или запуск AP fallback.
class WifiManager {
public:
    void begin(const String &ssid, const String &password);
    void startAccessPoint(const String &ssid, const String &password);
    bool isConnected() const;
    String localIP() const;

private:
    void connect(const String &ssid, const String &password, unsigned long timeoutMs = 10000);
};

#endif // WIFI_MANAGER_H
