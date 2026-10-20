// KOTELPC — точка входа. FreeRTOS-задачи + LittleFS + драйверы GPIO через HAL.
//
// Карта железа (конфигурация — только в коде драйвера):
//   IO21 = SDA MCP23017, IO22 = SCL MCP23017                                  (mcp23017_config.h)
//   IO16 = HEART pulse LED                     (gpio_config.h)
//   IO1  = UART TX -> RS485 DI                 (rs485_config.h)
//   IO3  = UART RX <- RS485 RO                 (rs485_config.h)
//   IO23 = RE/DE направление передачи RS485    (rs485_config.h)
//   MCP23017: GPA0..GPA7 = реле RL0..RL7, GPB0..GPB7 = входы DI0..DI7 (дебаунс)
//
// Канал RS485 работает в одном из двух режимов, переключаемых НА ЛЕТУ:
//   SIMPLE_UART — простой потоковый UART (текстовые команды, отладка);
//   MODBUS_RTU  — кадры с автоматическим RE/DE, межсимвольным t3.5 и CRC16.
// Команды канала (SIMPLE_UART): "mode modbus", "mode uart", "mode?", "ping".

#include <Arduino.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
#include "esp_err.h"
#include "project_config.h"
#include "hal/hal_gpio.h"
#include "hal/hal_mcp23017.h"
#include "hal/hal_rs485.h"
#include "hal/hal_uart_bridge.h"
#include "driver_gpio.h"
#include "mcp23017_config.h"
#include "rs485_config.h"
#include "fs_driver.h"
#include "freertos_task.h"
#include "wifi_manager.h"
#include "config_service.h"
#include "web_server.h"

// Прототипы вспомогательных функций уровня приложения (определения ниже)

// Heartbeat для диагностики зависаний через Watchdog Timer (0.6.2):
// каждая прикладная задача вызывает его в своём цикле; обёртка над
// esp_task_wdt_reset() определена в секции WDT-монитора ниже.
inline void wdtKickNow();

// ---- Задача heartbeat: светодиод HEART на IO16, выход, 1 с вкл / 1 с выкл ----
// Светодиод активный-низкий: включение = лог.0 на IO16 (драйвер сам инвертирует
// полярность по флагу activeLow в gpio_config.h). Период задаётся
// HEARTBEAT_PERIOD_MS (2 * HEARTBEAT_HALF_PERIOD_MS = 2 с): ровно 1 с горим,
// 1 с гаснем (скважность 50%).
class HeartbeatTask : public os::TaskBase {
protected:
    void run() override {
        bool on = false;
        while (isRunning()) {
            wdtKickNow(); // heartbeat для диагностики зависаний (0.6.2)
            on = !on;
            hal::gpioWrite(static_cast<int>(GpioPin::LED_HEART), on);  // true = LED lit (LOW on pin)
            delayMs(HEARTBEAT_HALF_PERIOD_MS);
        }
    }
};

// ---- Прикладная логика: одна переменная входы, одна выходы ----
// g_diState — стабилизированное дебаунсом состояние входов DI0..DI7
// (бит i = 1 => контакт DIi замкнут на GND). Обновляется каждые
// DI_POLL_INTERVAL_MS задачей AppLogicTask через драйверную машину дебаунса.
// g_doState — слово выходов RL0..RL7 (бит i = 1 => реле RLi включено).
// Записывается в порт A только при изменении (без лишних I2C-транзакций).
// Сейчас выход равен входу (зеркало) — место для реальной логики котла:
// достаточно заменить строку assign между обработкой и выводом.
volatile uint8_t g_diState = 0x00; // входы (только чтение из других задач)
volatile uint8_t g_doState = 0x00; // выходы

class AppLogicTask : public os::TaskBase {
protected:
    void run() override {
        delayMs(APP_START_DELAY_MS); // ждём завершения инициализации железа
        uint8_t lastLoggedDi = 0xFF, lastLoggedDo = 0xFF;
        while (isRunning()) {
            wdtKickNow(); // heartbeat для диагностики зависаний (0.6.2)
            if (hal::mcpIsReady()) {
                // 1) ВХОДЫ: пакетное чтение GPIOB + дебаунс (DEBOUNCE_MS)
                hal::mcpPollInputs();
                const uint8_t di = hal::mcpDebouncedState();
                if (di != g_diState) g_diState = di;

                // 2) ОБРАБОТКА (прикладная логика): пока зеркало DI -> DO
                const uint8_t doNew = di;

                // 3) ВЫХОДЫ: запись в порт A только при изменении
                if (doNew != g_doState) {
                    if (hal::mcpWritePortA(doNew)) {
                        g_doState = doNew;
                    }
                }
                if (di != lastLoggedDi || g_doState != lastLoggedDo) {
                    LOGP("[APP] DI=0x%02X -> DO=0x%02X\n", di, g_doState);
                    lastLoggedDi = di;
                    lastLoggedDo = g_doState;
                }
                // Снимок для веб-интерфейса (вкладка "Входы / Выходы")
                web::setDiDo(g_diState, g_doState, hal::mcpIsReady());
            }
            delayMs(DI_POLL_INTERVAL_MS);
        }
    }
};

// ---- Задача FS: проверка смонтированного LittleFS, запись лога ----
class FsLogTask : public os::TaskBase {
protected:
    void run() override {
        uint32_t n = 0;
        while (isRunning()) {
            wdtKickNow(); // heartbeat для диагностики зависаний (0.6.2)
            char line[64];
            snprintf(line, sizeof(line), "[%lus] fs alive, heap=%u\n",
                     (unsigned long)(xTaskGetTickCount() / configTICK_RATE_HZ),
                     (unsigned)ESP.getFreeHeap());
            fs::FsDriver::instance().appendFile("/log/system.log", line);
            if (++n % 30 == 0) {
                size_t total = 0, used = 0;
                if (fs::FsDriver::instance().spaceInfo(total, used)) {
                    LOGP("[FS] %zu/%zu bytes used\n", used, total);
                }
            }
            delayMs(FS_LOG_INTERVAL_MS);
        }
    }
};

// ---- Задача RS485: приём кадра/потока, обработка команд, режим на лету ----
class Rs485Task : public os::TaskBase {
protected:
    void run() override {
        uint8_t buf[128];
        while (isRunning()) {
            if (hal::rsMode() == RsMode::MODBUS_RTU) {
                // Кадровый режим: ждём кадр до таймаута, CRC проверяется драйвером
                size_t n = hal::rsReceiveFrame(buf, sizeof(buf), MODBUS_RESP_TIMEOUT_MS);
                if (n >= 4) {
                    LOGP("[RS485/MB] rx frame %u bytes:", (unsigned)n);
                    for (size_t i = 0; i < n && i < 16; ++i) LOGP(" %02X", buf[i]);
                    LOGP("\n");
                    // Здесь размещается клиентская логика Modbus (опрос приборов).
                    // Пример демо-запроса (холдинг-регистр 0, слейв 1):
                    // const uint8_t req[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
                    // hal::rsSendFrame(req, sizeof(req));
                }
                delayMs(50);
            } else {
                // Простой режим: читаем поток и разбираем текстовые команды
                size_t n = hal::rsRead(buf, sizeof(buf) - 1, 100);
                if (n == 0) continue;
                buf[n] = '\0';
                handleText((char *)buf);
            }
        }
    }

    static void handleText(char *s) {
        // Trim пробелов/перевода строки
        while (*s == ' ' || *s == '\r' || *s == '\n') s++;
        size_t len = strlen(s);
        while (len > 0 && (s[len-1] == '\r' || s[len-1] == '\n' || s[len-1] == ' ')) s[--len] = 0;
        if (len == 0) return;

        LOGP("[RS485/UART] cmd: \"%s\"\n", s);
        if (strncmp(s, "mode modbus", 11) == 0) {
            hal::rsSetMode(RsMode::MODBUS_RTU);
            LOGP("[RS485] mode -> MODBUS_RTU\n");
        } else if (strncmp(s, "mode uart", 9) == 0) {
            hal::rsSetMode(RsMode::SIMPLE_UART);
            LOGP("[RS485] mode -> SIMPLE_UART\n");
        } else if (strncmp(s, "mode?", 5) == 0) {
            LOGP("[RS485] mode = %s\n",
                 hal::rsMode() == RsMode::MODBUS_RTU ? "MODBUS_RTU" : "SIMPLE_UART");
        } else if (strncmp(s, "ping", 4) == 0) {
            static const char pong[] = "pong\n";
            hal::rsWrite((const uint8_t *)pong, sizeof(pong) - 1);
        }
    }
};

// ---- Задача инициализации сети: Wi-Fi + веб-сервер (boot-loop fix 0.6.2) ----
// Раньше g_wifi.begin() и web::start() вызывались прямо из setup(), т.е.
// исполнялись в контексте Arduino loopTask со стеком фиксированного размера.
// Инициализация радиостока (WiFi AP+STA, lwIP, DHCP, ESPAsyncWebServer)
// требовала больше стека, чем оставалось после LOGP-буферов и локальных
// объектов ConfigService/DeviceConfig (std::string) — происходило
// переполнение памяти и boot loop (режим "rst:0xC brownout... / abort() ->
// перезагрузка по кругу"). Теперь весь сетевой код живёт в выделенной
// FreeRTOS-задаче с увеличенным статическим стеком 8 КБ; стек задачи
// защищён MPU-stack overflow detection (при переполнении — чёткий дамп с
// именем "netboot", а не тихая порча памяти setup()).
class NetBootTask : public os::TaskBase {
protected:
    void run() override {
        LOGP("[NET] netboot task started (stack %d B, heap=%u)\n",
             NET_BOOT_TASK_STACK_BYTES, (unsigned)ESP.getFreeHeap());

        // 1. Wi-Fi: настройки из NVS (ConfigService); при пустом SSID или неудаче —
        //    точка доступа KOTELPC-AP / 12345678 (http://192.168.4.1/).
        //    Режим постоянный AP+STA (радио этой платы работает только с AP).
        {
            ConfigService svc;
            DeviceConfig cfg;
            if (!svc.load(cfg)) {
                cfg.wifiSsid = DEFAULT_WIFI_SSID;
                cfg.wifiPassword = DEFAULT_WIFI_PASSWORD;
            }
            g_wifi.begin(cfg.wifiSsid.c_str(), cfg.wifiPassword.c_str());
            LOGP("[WIFI] ap=%s sta=%s ssid=\"%s\" ip=%s\n",
                 AP_SSID,
                 g_wifi.isConnected() ? "connected" : "down",
                 g_wifi.currentSsid().c_str(),
                 g_wifi.localIP().c_str());
        }

        // 2. Веб-интерфейс (порт 80): вкладки Сеть / Входы-Выходы / Файлы LittleFS
        web::start();

        LOGP("[NET] network stack up, heap=%u\n", (unsigned)ESP.getFreeHeap());
        // Инициализация завершена — задача сама завершается (освобождает TCB и
        // статический стек остаются живы до delete экземпляра — он static).
    }
};

// ---- Диагностика Watchdog Timer (boot-loop / зависания) ----
// Прикладные задачи регистрируются в esp_task_wdt (TWDT) с таймаутом
// TASK_WDT_TIMEOUT_MS; panic/reboot по WDT ВЫКЛЮЧЕН (trigger_panic=false) —
// реализована именно ДИАГНОСТИКА: при просрочке heartbeat'а ядро дёргает
// слабый ISR-хук esp_task_wdt_isr_user_handler(), монитор забирает событие,
// печатает ИМЕНА зависших задач (esp_task_wdt_print_triggered_tasks + dump
// uxTaskGetSystemState с остатком стека) и НЕ перезагружает устройство.
// Задачи кормят собаку через wdtKickNow() (обёртка esp_task_wdt_reset()).
static const char *const kWatchedTasks[] = {"hb", "app", "fslog", "rs485"};

inline void wdtKickNow() { esp_task_wdt_reset(); }

static bool wdtSubscribeAll() {
    bool subscribed = false;
    for (const char *want : kWatchedTasks) {
        TaskHandle_t h = xTaskGetHandle(want); // NULL, если задача ещё не стартовала
        if (!h) continue;
        const esp_err_t err = esp_task_wdt_add(h); // уже подписана -> INVALID_STATE
        if (err == ESP_OK) {
            LOGP("[WDT] watching task \"%s\" (timeout %d ms)\n",
                 want, TASK_WDT_TIMEOUT_MS);
            subscribed = true;
        }
    }
    return subscribed;
}

// Прямой дамп списка задач: имя + оставшийся свободный стек (в словах).
// По имени сразу видно, какая подсистема стала причиной зависания.
static void wdtDumpTasks() {
    static TaskStatus_t st[16];
    UBaseType_t n = uxTaskGetSystemState(st, 16, nullptr);
    for (UBaseType_t i = 0; i < n; ++i) {
        LOGP("[WDT]   task \"%s\": prio=%u, stack watermark=%u words\n",
             st[i].pcTaskName, (unsigned)st[i].uxCurrentPriority,
             (unsigned)st[i].usStackHighWaterMark);
    }
}

// Флаг из ISR TWDT (устанавливается в слабом обработчике ниже), semaphore
// монитора — пробуждение для печати имён проблемных задач.
static volatile bool s_wdtFired = false;
static SemaphoreHandle_t s_wdtSem = nullptr;

// Слабый хук ESP-IDF вызывается из ISR при таймауте TWDT. Переопределяем его
// только для сигнала монитору (в ISR лог писать нельзя).
extern "C" void esp_task_wdt_isr_user_handler(void) {
    s_wdtFired = true;
    if (s_wdtSem) {
        BaseType_t hi = pdFALSE;
        xSemaphoreGiveFromISR(s_wdtSem, &hi);
        portYIELD_FROM_ISR(hi);
    }
}

class WdtMonitorTask : public os::TaskBase {
protected:
    void run() override {
        s_wdtSem = xSemaphoreCreateBinary();
        // Инициализация TWDT: таймаут TASK_WDT_TIMEOUT_MS, idle-задачи не
        // контролируем, trigger_panic=false => диагностика БЕЗ reboot/panic
        // (исключаем новый boot loop из-за ложных срабатываний собаки).
        esp_task_wdt_config_t cfg = {};
        cfg.timeout_ms = TASK_WDT_TIMEOUT_MS;
        cfg.idle_core_mask = 0;
        cfg.trigger_panic = false;
        const esp_err_t initErr = esp_task_wdt_init(&cfg); // INVALID_STATE = уже инициализирован
        if (initErr != ESP_OK && initErr != ESP_ERR_INVALID_STATE) {
            LOGP("[WDT] init failed: %s\n", esp_err_to_name(initErr));
            return;
        }
        esp_task_wdt_add(nullptr); // подписать саму задачу монитора (кормит каждый цикл)
        LOGP("[WDT] monitor started (timeout %d ms, no-reboot diagnostics)\n",
             TASK_WDT_TIMEOUT_MS);

        while (isRunning()) {
            wdtSubscribeAll(); // подписывает прикладные задачи (hb/app/fslog/rs485)
            wdtKickNow();      // heartbeat самого монитора

            // Событие TWDT: зависла хотя бы одна подписанная задача.
            // Печатаем её ИМЯ (штатный вывод ядра + собственный dump задач).
            if (s_wdtFired) {
                s_wdtFired = false;
                int cpusFail = 0;
                LOGP("[WDT] *** task watchdog triggered (CPU mask=0x%X) — stuck tasks:\n",
                     (unsigned)cpusFail);
                // Штатный список задач, не сбросивших TWDT (имена печатает ядро):
                esp_task_wdt_print_triggered_tasks(nullptr, nullptr, &cpusFail);
                wdtDumpTasks();
            }
            // Ожидаем либо период опроса, либо сигнал из ISR (зависание).
            xSemaphoreTake(s_wdtSem, pdMS_TO_TICKS(TASK_WDT_CHECK_INTERVAL_MS));
        }
    }
};

static HeartbeatTask heartbeatTask;
static AppLogicTask appLogicTask;  // входы(g_diState) -> логика -> выходы(g_doState)
static FsLogTask fsLogTask;
static Rs485Task rs485Task;
static NetBootTask netBootTask;    // Wi-Fi + веб-сервер (увеличенный стек 8 КБ)
static WdtMonitorTask wdtMonTask;  // диагностика зависаний по имени задачи

void setup() {
    const uint32_t bootMs = millis(); // бюджет времени инициализации MCP (non-blocking)
    // Отладочный Serial (до инициализации моста логи идут сюда же)
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);

    LOGP("\nKOTELPC boot. FW version: %s\n", FW_VERSION);
    LOGP("Chip: %s, rev %d, cores %d, freq %d MHz\n",
         ESP.getChipModel(), ESP.getChipRevision(),
         ESP.getChipCores(), ESP.getCpuFreqMHz());
    LOGP("Pins: IO16=HEART LED(active LOW), IO4=PERIPH_EN, IO1=RS485 TX, "
         "IO3=RS485 RX, IO23=RE/DE, "
         "IO21=SDA, IO22=SCL -> MCP23017 @0x20 (ADDR=GND), "
         "GPA=RL0..RL7, GPB=DI0..DI7\n");

    // 1. Драйверы железа (конфигурация пинов — внутри драйвера)
    if (!hal::gpioInit()) {
        LOGP("[FATAL] GPIO driver init failed\n");
    }

    // 1a. Питание периферии: подаём логическую 1 на IO4 (PERIPH_EN),
    //     ждём стабилизации питания MCP23017/реле/RS485 перед их инициализацией.
    hal::gpioWrite(static_cast<int>(GpioPin::PERIPH_EN), true);
    delay(PERIPH_POWER_STABLE_MS);
    LOGP("[PWR] PERIPH_EN (IO4) = 1, periphery power on\n");

    // 1b. Расширитель GPIO MCP23017 по I2C (шина/адрес/порты — в mcp23017_config.h)
    //     Инициализация с повторными попытками: чипу после включения питания
    //     по PERIPH_EN нужно время на POR; шина при этом опрашивается заново.
    //     НЕУДАЧНАЯ ИНИЦИАЛИЗАЦИЯ НЕ БЛОКИРУЕТ СИСТЕМУ: дальнейшие обращения
    //     к чипу экранированы флагом готовности драйвера (ready_), задача
    //     AppLogicTask проверяет hal::mcpIsReady() и просто пропускает цикл.
    bool mcpOnline = false;
    for (int attempt = 1; attempt <= MCP_INIT_RETRIES && !mcpOnline; ++attempt) {
        if (hal::mcpInit()) {
            mcpOnline = true;
            LOGP("[MCP] MCP23017 online @0x%02X (sda=%d scl=%d), attempt %d\n",
                 mcp_config::I2C_ADDR, mcp_config::SDA_PIN, mcp_config::SCL_PIN, attempt);
        } else {
            LOGP("[MCP] init attempt %d/%d failed, retry in %d ms\n",
                 attempt, MCP_INIT_RETRIES, MCP_INIT_RETRY_DELAY_MS);
            delay(MCP_INIT_RETRY_DELAY_MS);
        }
        // Ограничение времени: инициализация расширителя НЕ должна блокировать
        // запуск остальных задач больше MCP_INIT_MAX_BLOCK_MS (мигание LED,
        // LittleFS, RS485 стартуют сразу после этого блока). Превысили бюджет —
        // выходим из цикла; чип при этом остаётся "offline".
        if (!mcpOnline &&
            millis() - bootMs >= (uint32_t)MCP_INIT_MAX_BLOCK_MS) {
            LOGP("[MCP] init budget (%d ms) exceeded, giving up for now (non-blocking)\n",
                 MCP_INIT_MAX_BLOCK_MS);
            break;
        }
    }
    if (!mcpOnline) {
        LOGP("[ERROR] MCP23017 не найден на шине I2C (адрес 0x20, ADDR=GND).\n"
             "        Проверьте: питание периферии (IO4), SDA=IO21/SCL=IO22,\n"
             "        подтяжки 4.7k к +3V3 на SDA/SCL, монтаж микросхемы.\n");
    }

    // 1c. Канал RS485: UART + RE/DE (конфигурация — rs485_config.h).
    //     Arduino HWSerial (Serial) использует UART0 — освобождаем порт
    //     перед установкой driver/uart поверх тех же пинов.
    Serial.end();
    if (hal::rsInit()) {
#if RS485_START_MODBUS
        hal::rsSetMode(RsMode::MODBUS_RTU);
#endif
        LOGP("[RS485] ready @%u baud, mode=%s\n",
             (unsigned)rs485_config::BAUD_RATE,
             hal::rsMode() == RsMode::MODBUS_RTU ? "MODBUS_RTU" : "SIMPLE_UART");
    } else {
        LOGP("[FATAL] RS485 driver init failed\n");
    }
    // Мост логов/ввода: отладочный вывод можно перенаправить в шину RS485
    // (hal::uartToRs485(true)) или вернуть в Serial (hal::uartToRs485(false)).
    hal::uartBridgeInit(UART_BRIDGE_TO_RS485);

    // 2. LittleFS (раздел "storage" из partitions.csv)
    if (!fs::FsDriver::instance().begin(true)) {
        LOGP("[WARN] LittleFS mount failed\n");
    } else {
        fs::FsDriver::instance().mkdir("/log");
        fs::FsDriver::instance().mkdir("/www");   // каталог файлов веб-интерфейса
    }

    // 2a/2b. Wi-Fi и веб-сервер — НЕ в setup(): инициализация сетевого стека
    //     (WiFi AP+STA, lwIP, ESPAsyncWebServer) перенесена в отдельную
    //     FreeRTOS-задачу "netboot" со стеком 8 КБ (boot-loop fix 0.6.2).
    //     В контексте loopTask/setup() её стек не вмещался — переполнение
    //     памяти вызывало boot loop. Задача сама завершается после старта сети.
    netBootTask.start(os::TaskBase::Config{
        "netboot", NET_BOOT_TASK_STACK_BYTES, NET_BOOT_TASK_PRIORITY, tskNO_AFFINITY});

    // 3. Прикладные FreeRTOS-задачи
    heartbeatTask.start(os::TaskBase::Config{"hb", 3072, tskIDLE_PRIORITY + 1, tskNO_AFFINITY});
    appLogicTask.start(os::TaskBase::Config{"app", 3072, tskIDLE_PRIORITY + 2, tskNO_AFFINITY});
    fsLogTask.start(os::TaskBase::Config{"fslog", 4096, tskIDLE_PRIORITY + 1, tskNO_AFFINITY});
    rs485Task.start(os::TaskBase::Config{"rs485", 4096, tskIDLE_PRIORITY + 2, tskNO_AFFINITY});

    // 4. Монитор Watchdog Timer: подписывает задачи на esp_task_wdt и при
    //    просрочке heartbeat позволяет ядру напечатать имя зависшей задачи.
    wdtMonTask.start(os::TaskBase::Config{"wdtmon", 3072, tskIDLE_PRIORITY + 1, tskNO_AFFINITY});

    LOGP("[MAIN] tasks started\n");
}

void loop() {
    // В Arduino-каркасе loop() сам по себе является задачей FreeRTOS.
    // Вся полезная работа вынесена в задачи выше.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
