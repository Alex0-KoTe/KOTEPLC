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
#define FW_VERSION "0.3.4"

// ---- Питание периферии ----
// PERIPH_EN (IO4): логическая 1 включает питание всей периферии
// (MCP23017, реле, RS485). Пауза до инициализации драйверов —
// время стабилизации напряжения после подачи.
// Увеличено до 300 мс: при питании через ключ (IO4) ёмкости на нагрузке
// заряжаются медленно, и MCP23017 может ещё не выйти из POR к моменту
// первого обращения к шине.
#define PERIPH_POWER_STABLE_MS 300

// ---- Инициализация MCP23017 ----
// Повторные попытки после включения питания периферии (POR микросхемы).
#define MCP_INIT_RETRIES        8
#define MCP_INIT_RETRY_DELAY_MS 400

// ---- FreeRTOS-периоды (мс) ----
#define HEARTBEAT_PERIOD_MS   1000   // мигание статусным LED
#define INPUT_POLL_INTERVAL_MS 20    // опрос кнопок/входов
#define BUTTON_DEBOUNCE_MS     50    // дебаунс кнопки
#define FS_LOG_INTERVAL_MS     1000  // запись в лог LittleFS
#define MCP_POLL_INTERVAL_MS   100   // (резерв) период фоновых операций MCP
#define DI_POLL_INTERVAL_MS    10    // опрос входов DI0..DI7 + дебаунс (мс)

// ---- Однократный тест реле при старте (проверка железа) ----
// Задача по очереди щёлкает всеми реле RL0..RL7 с частотой 2 Гц
// (полупериод 250 мс на реле), затем выключает все и выдаёт в простой UART
// сообщение "Тест реле пройден".
#define RELAY_TEST_ENABLED      1
#define RELAY_TEST_FREQ_HZ      2      // частота переключения, Гц
#define RELAY_TEST_HALFPERIOD_MS (1000 / RELAY_TEST_FREQ_HZ / 2)  // 250 мс
#define RELAY_TEST_START_DELAY_MS 2000 // задержка старта после инициализации железа

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
