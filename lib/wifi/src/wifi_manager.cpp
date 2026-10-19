#include "wifi_manager.h"
#include "project_config.h"
#include <Arduino.h>

WifiManager g_wifi;   // единый экземпляр менеджера для всего приложения

void WifiManager::begin(const String &ssid, const String &password) {
    // Постоянный режим работы этой платы: AP+STA (см. startAccessPoint).
    // 1) Всегда поднимаем точку доступа KOTELPC-AP — веб-интерфейс доступен
    //    по http://192.168.4.1/ независимо от наличия настроенной сети.
    startAccessPoint(AP_SSID, AP_PASSWORD);
    // 2) Если сеть задана — подключаемся STA-клиентом сверху (не более
    //    timeoutMs; неудача не блокирует систему — драйвер сам продолжает
    //    переподключение, а пользователь может выбрать сеть в веб-UI).
    if (ssid.length() > 0) {
        connect(ssid, password);
        apMode_ = false;   // статус в UI: connected/rssi определяются isConnected()
        ssid_ = ssid;      // показываем целевую STA-сеть
    }
}

void WifiManager::connect(const String &ssid, const String &password, unsigned long timeoutMs) {
    WiFi.persistent(false);
    // Режим уже AP_STA (поднят в startAccessPoint); гарантируем его на случай
    // вызова connect() напрямую.
    if (WiFi.getMode() != WIFI_AP_STA) WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);            // стабильность latency для веб-интерфейса
    WiFi.begin(ssid.c_str(), password.c_str());

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
        vTaskDelay(pdMS_TO_TICKS(250));   // уступаем CPU служебным задачам стека Wi-Fi
    }
}

void WifiManager::startAccessPoint(const String &ssid, const String &password) {
    // На этой плате Wi-Fi работает ТОЛЬКО в режиме WIFI_AP (проверено на железе:
    // в чистом WIFI_STA радио не поднимается). Поэтому AP+STA — постоянный
    // режим работы: точка доступа KOTELPC-AP доступна всегда, а STA добавляется
    // сверху, когда задана сеть.
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);
    WiFi.softAP(ssid.c_str(), password.c_str());
    delay(120);   // дать драйверу поднять AP перед возможными STA-операциями
    apMode_ = true;
    ssid_ = ssid;
}

bool WifiManager::isConnected() const {
    // AP поднят всегда (AP+STA); «подключён» = есть STA-соединение.
    return WiFi.status() == WL_CONNECTED;
}

String WifiManager::localIP() const {
    String ip = WiFi.localIP().toString();
    if (apMode_ || WiFi.status() != WL_CONNECTED) {
        ip = WiFi.softAPIP().toString();   // 192.168.4.1 — адрес точки доступа
    }
    return ip;
}
