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
#define FW_VERSION "0.6.4"

// ---- Boot-loop fix (0.6.2/0.6.3): Wi-Fi + веб-сервер в отдельной задаче ----
// Инициализация стека Wi-Fi (lwIP/wpa_supplicant) и AsyncWebServer требует
// глубокого стека; запуск их из setup() (контекст Arduino loopTask, стек
// CONFIG_ARDUINO_LOOP_STACK_SIZE) приводил к переполнению памяти и boot loop.
// Теперь весь сетевой стек стартует в выделенной FreeRTOS-задаче "netboot".
// ВНИМАНИЕ (0.6.3): стек задачи выделяется СТАТИЧЕСКИ (xTaskCreateStatic,
// буфер std::vector<StackType_t> живёт в куче всё время жизни задачи).
// 8 КБ стека + TCB — это ~8.5 КБ непрерывного DRAM-блока на раннем этапе
// загрузки, когда Wi-Fi/lwIP-пул ещё не аллокирован и фрагментация минимальна,
// поэтому 8 КБ безопасны. УМЕНЬШАТЬ ЭТОТ РАЗМЕР НЕЛЬЗЯ: при переполнении
// стека netboot происходит exactly то же повреждение памяти и boot loop.
#define NET_BOOT_TASK_STACK_BYTES 8192
#define NET_BOOT_TASK_PRIORITY    (tskIDLE_PRIORITY + 3)

// ---- Watchdog Timer (диагностика зависаний) ----
// Периодический опрос задач через esp_task_wdt: если задача не сообщает о
// себе более TASK_WDT_TIMEOUT_MS, WDT печатает dump всех задач — по имени
// "stuck" задачи видно, какая именно зависла (до сброса, если включён panic).
#define TASK_WDT_TIMEOUT_MS       10000
#define TASK_WDT_CHECK_INTERVAL_MS 1000

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
// Максимальное время, которое инициализация MCP23017 может блокировать setup()
// (задержка старта мигания LED и остальных задач). По истечении бюджета цикл
// попыток прерывается, система продолжает запуск, чип остаётся offline.
#define MCP_INIT_MAX_BLOCK_MS   3500

// ---- FreeRTOS-периоды (мс) ----
// Heartbeat LED: ровно 1 с горит / 1 с гаснет (скважность 50%).
// Период = 2 * полупериод = 2 с.
#define HEARTBEAT_HALF_PERIOD_MS 1000  // 1 с горит / 1 с гаснет (50% duty)
#define HEARTBEAT_PERIOD_MS   (2 * HEARTBEAT_HALF_PERIOD_MS)   // мигание статусным LED
#define INPUT_POLL_INTERVAL_MS 20    // опрос кнопок/входов
#define BUTTON_DEBOUNCE_MS     50    // дебаунс кнопки
#define FS_LOG_INTERVAL_MS     1000  // запись в лог LittleFS
#define MCP_POLL_INTERVAL_MS   100   // (резерв) период фоновых операций MCP

// ---- Прикладная логика (входы -> выходы) ----
// Задача AppLogicTask: периодическое чтение входов DI0..DI7 с дебаунсом
// (машина дебаунса в драйвере MCP23017, DEBOUNCE_MS) в g_diState и запись
// слова выходов g_doState в реле RL0..RL7 (порт A). Период цикла опроса:
#define DI_POLL_INTERVAL_MS    10    // опрос входов DI0..DI7 + дебаунс (мс)
#define APP_START_DELAY_MS     2000  // задержка старта после инициализации железа

// Настройки последовательного порта отладки
#define SERIAL_BAUD_RATE 115200

// ---- Канал RS485 (конфигурация пинов — в драйвере, rs485_config.h) ----
// Стартовый режим канала: 0 = SIMPLE_UART (текстовые команды), 1 = MODBUS_RTU.
// Переключить на лету можно командой "mode modbus"/"mode uart" из шины.
#define RS485_START_MODBUS        0
// Таймаут ожидания ответа/кадра Modbus, мс
#define MODBUS_RESP_TIMEOUT_MS    500
// Куда идут логи/ввод после старта: 0 = Serial (отладка), 1 = шина RS485.
// ВНИМАНИЕ (boot-loop fix 0.6.4): значение по умолчанию — 1, потому что
// транслятор RS485 физически висит на UART0 = отладочный Serial. После
// hal::rsInit() портом владеет драйвер RS485; писать в тот же UART через
// объект Serial нельзя (двойное владение = мусор в потоке и boot loop).
// 0 допустимо ТОЛЬКО если RS485-транслятор физически отключён/не инициализирован.
#define UART_BRIDGE_TO_RS485      1

#endif // PROJECT_CONFIG_H
