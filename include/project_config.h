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
#define FW_VERSION "0.1.0"

// Пин встроенного светодиода для индикации состояния
#define STATUS_LED_PIN 2

// Интервалы работы (мс)
#define LOOP_DELAY_MS 10
#define SENSOR_READ_INTERVAL_MS 1000

// Настройки последовательного порта отладки
#define SERIAL_BAUD_RATE 115200

#endif // PROJECT_CONFIG_H
