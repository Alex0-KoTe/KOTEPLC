#include "wifi_manager.h"
#include "project_config.h"
#include <Arduino.h>

void WifiManager::begin(const String &ssid, const String &password) {
    if (ssid.length() == 0) {
        startAccessPoint(AP_SSID, AP_PASSWORD);
        return;
    }
    connect(ssid, password);
    if (!isConnected()) {
        // Fallback: поднимаем точку доступа
        startAccessPoint(AP_SSID, AP_PASSWORD);
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
}

bool WifiManager::isConnected() const {
    return WiFi.status() == WL_CONNECTED;
}

String WifiManager::localIP() const {
    return WiFi.localIP().toString();
}
