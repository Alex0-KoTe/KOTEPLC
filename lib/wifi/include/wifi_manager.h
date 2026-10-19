#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include <WString.h>

// Простой менеджер Wi-Fi: подключение к точке или запуск AP fallback.
class WifiManager {
public:
    void begin(const String &ssid, const String &password);
    void startAccessPoint(const String &ssid, const String &password);
    bool isConnected() const;
    String localIP() const;
    bool isApMode() const { return apMode_; }       // true = работает точка доступа
    String currentSsid() const { return ssid_; }    // STA: сеть подключения; AP: имя AP

private:
    void connect(const String &ssid, const String &password, unsigned long timeoutMs = 10000);
    bool apMode_ = false;
    String ssid_;
};

extern WifiManager g_wifi;   // единый экземпляр менеджера (определён в wifi_manager.cpp)

#endif // WIFI_MANAGER_H
