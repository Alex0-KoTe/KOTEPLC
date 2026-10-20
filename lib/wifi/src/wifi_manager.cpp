#include "wifi_manager.h"
#include "project_config.h"
#include <Arduino.h>

WifiManager g_wifi;   // единый экземпляр менеджера для всего приложения

void WifiManager::begin(const String &ssid, const String &password) {
    // 1) Всегда поднимаем ТОЧКУ ДОСТУПА KOTELPC-AP — веб-интерфейс доступен
    //    по http://192.168.4.1/ (0.6.9: чистый WIFI_AP, как в эталонной
    //    прошивке пользователя; STA на старте не поднимается).
    startAccessPoint(AP_SSID, AP_PASSWORD);
    // 2) STA — только если сеть реально задана (пользователь сохранил её
    //    через веб-UI). Пустой SSID = никакого WiFi.begin() на старте.
    if (ssid.length() > 0) {
        connect(ssid, password);
        apMode_ = false;   // статус в UI: connected/rssi определяются isConnected()
        ssid_ = ssid;      // показываем целевую STA-сеть
    }
}

void WifiManager::connect(const String &ssid, const String &password, unsigned long timeoutMs) {
    WiFi.persistent(false);
    // Ленивый STA (0.6.9): поднимаем AP_STA только сейчас — по явному запросу
    // пользователя из веб-UI. Вызывается исключительно из контекста loopTask
    // (web::handleLoopTick), что безопасно для блокировок lwIP/esp_wifi.
    if (WiFi.getMode() != WIFI_AP_STA) WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);            // стабильность latency для веб-интерфейса
    WiFi.begin(ssid.c_str(), password.c_str());

    // Неблокирующее ожидание (0.6.9): раньше здесь крутился while с
    // vTaskDelay до 10 секунд — это «вешало» loopTask, останавливало обработку
    // TCP/lwIP-очередей и выглядело как зависание при отсутствии сети.
    // Статус подключения приложение опрашивает через isConnected().
    (void)timeoutMs;
}

void WifiManager::startAccessPoint(const String &ssid, const String &password) {
    // На этой плате Wi-Fi работает ТОЛЬКО в режиме WIFI_AP (проверено на железе:
    // в чистом WIFI_STA радио не поднимается). FW 0.6.9: стартап = ЧИСТЫЙ
    // WIFI_AP (как в эталонной прошивке пользователя, где AP работает без
    // единого сбоя). Режим AP_STA больше НЕ выставляется здесь: переход
    // драйвера в AP_STA + WiFi.begin() по несуществующей сети был источником
    // тихого boot loop (крах сразу после nvs_open, маркер last step=7 так и
    // не записывался). STA включается ЛЕНИВО — только когда пользователь
    // реально задал/выбрал сеть через веб-UI (connect()).
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP);
    // 0.6.8: в arduino-esp32 3.x раскладка ядер задаётся на этапе сборки
    // (CONFIG_ARDUINO_RUNNING_CORE=1, CONFIG_ESP_WIFI_TASK_CORE_ID=0) —
    // рантайм-API setCore()/radio_task_on_cpu() здесь недоступен. Штатная
    // раскладка уже корректна: loopTask/AsyncTCP/lwIP на core 1, задача
    // esp_wifi на core 0. Никаких WiFi-вызовов из прикладных задач!
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
