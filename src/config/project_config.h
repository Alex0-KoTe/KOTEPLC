#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

// ---- Общие настройки проекта KOTELPC (ESP32) ----

// Имя точки доступа (AP mode fallback)
#define AP_SSID "KOTELPC-AP"
#define AP_PASSWORD "12345678"

// Параметры Wi-Fi по умолчанию (переопределяются хранимой конфигурацией)
#define DEFAULT_WIFI_SSID ""
#define DEFAULT_WIFI_PASSWORD ""

// Версия прошивки
#define FW_VERSION "0.3.0"

// ---- FreeRTOS-периоды (мс) ----
#define HEARTBEAT_PERIOD_MS   1000   // мигание статусным LED
#define INPUT_POLL_INTERVAL_MS 20    // опрос кнопок/входов
#define BUTTON_DEBOUNCE_MS     50    // дебаунс кнопки
#define FS_LOG_INTERVAL_MS     1000  // запись в лог LittleFS
#define MCP_POLL_INTERVAL_MS   100   // опрос расширителя MCP23017

// Настройки последовательного порта отладки
#define SERIAL_BAUD_RATE 115200

#endif // PROJECT_CONFIG_H
