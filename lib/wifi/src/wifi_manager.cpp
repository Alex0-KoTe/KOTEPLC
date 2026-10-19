#include "wifi_manager.h"
#include "project_config.h"
#include <Arduino.h>

WifiManager g_wifi;   // единый экземпляр менеджера для всего приложения

void WifiManager::begin(const String &ssid, const String &password) {
    if (ssid.length() == 0) {
        startAccessPoint(AP_SSID, AP_PASSWORD);
        return;
    }
    connect(ssid, password);
    if (!isConnected()) {
        // Fallback: поднимаем точку доступа
        startAccessPoint(AP_SSID, AP_PASSWORD);
    } else {
        apMode_ = false;
        ssid_ = ssid;
    }
}

void WifiManager::connect(const String &ssid, const String &password, unsigned long timeoutMs) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
        delay(250);
    }
}

void WifiManager::startAccessPoint(const String &ssid, const String &password) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid.c_str(), password.c_str());
    apMode_ = true;
    ssid_ = ssid;
}

bool WifiManager::isConnected() const {
    return apMode_ ? true : (WiFi.status() == WL_CONNECTED);
}

String WifiManager::localIP() const {
    return apMode_ ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
}
