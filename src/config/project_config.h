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
#define MCP_POLL_INTERVAL_MS   100   // (резерв) период фоновых операций MCP
#define DI_POLL_INTERVAL_MS    10    // опрос входов DI0..DI7 + дебаунс (мс)
#define RELAY_TEST_INTERVAL_MS 500   // тестовая последовательность реле RL0..RL7 (мс)

// Настройки последовательного порта отладки
#define SERIAL_BAUD_RATE 115200

// ---- Канал RS485 (конфигурация пинов — в драйвере, rs485_config.h) ----
// Стартовый режим канала: 0 = SIMPLE_UART (текстовые команды), 1 = MODBUS_RTU.
// Переключить на лету можно командой "mode modbus"/"mode uart" из шины.
#define RS485_START_MODBUS        0
// Таймаут ожидания ответа/кадра Modbus, мс
#define MODBUS_RESP_TIMEOUT_MS    500
// Куда идут логи/ввод после старта: 0 = Serial (отладка), 1 = шина RS485
#define UART_BRIDGE_TO_RS485      0

#endif // PROJECT_CONFIG_H
