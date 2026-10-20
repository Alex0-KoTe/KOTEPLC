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
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
#include "esp_err.h"
#include "esp_netif.h"   // 0.6.11: esp_netif_is_netif_up (прайминг tcpip_thread)
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

// FW 0.6.12 (boot-loop fix): флаг готовности ядра TCP/IP. Выставляется по
// событию ARDUINO_EVENT_WIFI_AP_START (штатное событие режима стартапа этой
// платы — чистый WIFI_AP) либо fallback'ом по факту поднятого AP netif.
// Раньше готовность определялась событием STA_START, которое в AP-only
// стартапе не приходит никогда — ожидание уходило в таймаут, трактовался
// как фатальный и заканчивался перезагрузкой (SW_CPU_RESET).
static volatile bool s_tcpipReady = false;

// Прототипы вспомогательных функций уровня приложения (определения ниже)

// Heartbeat для диагностики зависаний через Watchdog Timer (0.6.2):
// каждая прикладная задача вызывает его в своём цикле; обёртка над
// esp_task_wdt_reset() определена в секции WDT-монитора ниже.
inline void wdtKickNow();

// Маркер шагов инициализации (boot-loop bisect, 0.6.4) — определения ниже,
// используются и в NetBootTask, и в setup().
static void dbgStepMark(uint8_t step);

// Инициализация канала RS485 (FW 0.6.7: вынесена из setup() и вызывается
// ПОСЛЕ старта сети; определение — после объявлений задач ниже).
static void initRs485Channel();

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
// 0.6.6: тело задачи вынесено в обычную функцию, которую setup() вызывает
// ПРЯМО (в контексте loopTask). Причина: маркер «last step=7» в boot loop
// был ложным — он писался ПОСЛЕ успешного begin(), то есть показывал результат
// ПРЕДЫДУЩего цикла, а крах происходил ВНУТРИ WiFi.mode()/softAP(). Заодно
// убран конкурент netboot со стеком 8 КБ и приоритетом выше wifi task —
// на старте радио такая связка деградировала Wi-Fi-стек.
static void startNetworkStack() {
    LOGP("[NET] network init start (heap=%u, maxblock=%u)\n",
         (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());

    // 1. Только ТОЧКА ДОСТУПА (режим WIFI_AP) — ровно как в эталонной
    //    прошивке пользователя, где «WiFi AP работает прекрасно» (0.6.9).
    //    STA-часть (WIFI_AP_STA + WiFi.begin) убрана из стартапа полностью:
    //    именно она в 0.6.8 приводила к тихому краху сразу после
    //    Preferences/nvs_open (переход состояния драйвера при отключённом
    //    WiFi.persistent(false) не сохранял коннект и ронял систему).
    //    Подключение к домашней сети — исключительно по желанию через
    //    веб-UI (POST /api/network -> handleLoopTick в контексте loopTask).
    dbgStepMark(10); // точка невозврата: СЕЙЧАС поднимаем радио (AP ON)
    // 0.6.10: ПОЛНЫЙ A/B-эквивалент эталонной прошивки пользователя (в которой
    // «WiFi AP работает прекрасно»). Эталон делает ровно:
    //   WiFi.persistent(false); WiFi.mode(WIFI_AP); WiFi.softAP(ssid, pass);
    // — синхронно в setup(), БЕЗ setSleep(false), БЕЗ delay() после softAP,
    // БЕЗ отдельных задач. Любое отклонение здесь подозревается как причина
    // тихого boot loop (крах между [NET] start и маркером wifi begin OK).
    // Поэтому радио поднимается минимальным набором вызовов прямо отсюда;
    // WifiManager::startAccessPoint() остаётся для пути веб-UI.
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP);
    // FW 0.6.13: modemsleep ВЫКЛЮЧЕН сразу перед softAP — совпадает с
    // эталонной прошивкой, где AP поднимается без единого сбоя. Модуль сна
    // Wi-Fi (PSM/modem sleep) на этом железе давал пиковое потребление при
    // первом включении радио — косвенный кандидат на просадку 3V3 в момент
    // старта AP. Вызов идемпотентен; WifiManager::startAccessPoint() делает
    // то же самое на пути веб-UI.
    WiFi.setSleep(false);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    dbgStepMark(7); // Wi-Fi AP стартовал без краха
    LOGP("[WIFI] ap=%s ip=%s (mode=WIFI_AP, STA off)",
         AP_SSID, g_wifi.localIP().c_str());
    LOGP(" by design; connect via web UI)\n");

    // 2. Веб-интерфейс (порт 80): вкладки Сеть / Входы-Выходы / Файлы LittleFS
    // 0.6.11: сервер стартует ПОСЛЕ запуска прикладных задач (см. setup());
    // поднят ли он — сообщает web::isRunning() (флаг s_started).
    dbgStepMark(8); // очередь на старт веб-сервера

    // 3. TCP/IP «прогрев» (0.6.11) — КЛЮЧЕВОЕ исправление lwIP boot loop.
    // Диагноз по backtrace 0.6.10: крах НЕ в Wi-Fi и НЕ в AsyncTCP, а внутри
    // esp_netif_set_ip4_info() при ПЕРВОМ в мире обращении к ядру TCP/IP:
    //   tcp_alloc -> "Required to lock TCPIP core functionality!"
    // (tcpip_lock_tcp_core проверяет owner == xTaskGetCurrentTaskHandle();
    //  assert срабатывает, когда tcpip_thread ещё не зарегистрировал владельца
    //  блокировки). Причина: CONFIG_LWIP_TCPIP_TASK_AFFINITY=NO_AFFINITY —
    //  задача tcpip_thread создаётся ЛЕНИВО, при первом же tcpip_callback.
    //  На данном SDK она так и не успевала стартовать до вызова softAP(),
    //  поэтому owner мьютекса оставался NULL и первый же сетевой вызов из
    //  setup() ронял систему (тихо, без panic-дампа — abort->SW_CPU_RESET).
    //  Рабочая эталонная прошивка косвенно «тронула» стек TCP/IP раньше
    //  (например DHCPD при softAP) — поэтому её AP работает идеально.
    // Исправление: принудительно разбудить tcpip_thread ДО любых операций с
    // IP-конфигурацией. esp_netif_is_netif_up() вызывает netif->status_get,
    // который идёт через tcpip_callback -> гарантированно создаёт/крутит
    // tcpip_thread на его собственном стеке; блокировка корректно
    // регистрируется владельцем. Вызов идемпотентен и безопасен.
    // 0.6.12: результат прогрева (apUp) — fallback-доказательство готовности
    // ядра для флага s_tcpipReady (см. шаг 4 ниже).
    {
        esp_netif_t *apIf = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        bool apUp = false;
        if (apIf) {
            for (int i = 0; i < 5 && !apUp; ++i) {
                apUp = esp_netif_is_netif_up(apIf); // tcpip_callback => wake tcpip_thread
                if (!apUp) vTaskDelay(pdMS_TO_TICKS(50));
            }
            LOGP("[NET] tcpip core primed (ap netif up=%d)\n", (int)apUp);
        } else {
            LOGP("[NET] WARN: AP netif handle not found (default config?)\n");
        }
        if (apUp) s_tcpipReady = true; // fallback: AP поднят => ядро живо
    }
    dbgStepMark(9); // tcpip_thread разбужен, IP-операции безопасны

    // 4. FW 0.6.12 (boot-loop fix): флаг готовности TCP/IP-ядра выставляется
    //    по событию ARDUINO_EVENT_WIFI_AP_START вместо STA_START.
    //    Причина: стартап этой платы — чистый WIFI_AP (STA включается лениво,
    //    только из веб-UI), поэтому событие STA_START здесь не приходит
    //    НИКОГДА; любое ожидание «STA-события => ядро готово» в AP-only
    //    режиме неизбежно уходит в таймаут и заканчивается перезагрузкой
    //    (SW_CPU_RESET сразу после «[NET] network stack up»). AP_START —
    //    штатное событие нашего режима (см. лог: mode=WIFI_AP), оно и
    //    устанавливает s_tcpipReady. Обработчик регистрируется идемпотентно
    //    (флаг s_apEvtRegistered); если событие уже обработано ESP-аркой
    //    до регистрации, считается доказательством сам факт поднятого AP
    //    netif (apUp выше) — событийная очередь Arduino не хранит историю.
    static bool s_apEvtRegistered = false;
    if (!s_apEvtRegistered) {
        WiFi.onEvent([](arduino_event_id_t ev) {
            if (ev == ARDUINO_EVENT_WIFI_AP_START) s_tcpipReady = true;
        });
        s_apEvtRegistered = true;
    }
    // Дождаться диспатча AP_START в очереди событий esp_event. В этой сборке
    // Arduino-core 3.x у WiFiClass НЕТ метода run(): события доставляются
    // напрямую из esp_event_default_loop (_arduino_event_cb -> onEvent),
    // поэтому достаточно короткого ожидания под vTaskDelay — ядро-владелец
    // цикла событий крутится на системном стеке и обработчик сработает сам.
    // Таймаут НЕ фатален: прайминг tcpip_thread (шаг 3) уже выполнен, сеть
    // работоспособна.
    for (int i = 0; i < 20 && !s_tcpipReady; ++i) {
        vTaskDelay(pdMS_TO_TICKS(25)); // ждём delivery AP_START -> s_tcpipReady
    }
    LOGP("[NET] tcpip ready flag=%d (set by %s)\n", (int)s_tcpipReady,
         s_tcpipReady ? "AP_START" : "timeout, netif-up fallback holds");

    LOGP("[NET] network stack up, heap=%u\n", (unsigned)ESP.getFreeHeap());

    // FW 0.6.13 (boot-loop bisect): «маркер выживания» после завершения
    // setup(). Между "[NET] network stack up" и концом setup() в логе 0.6.12
    // не печатается НИ ОДНА строка, а следующий boot показывает EXT(4) —
    // т.е. сброс приходит ровно в этом окне (web::start + RS485 + задачи).
    // Маркер 12 пишется в NVS ПОСЛЕ всего этого окна: если при следующем
    // старте last step=12 — код проходит setup до конца, и виновник живёт
    // в loop()/пост-setup; если last step остаётся 6 — крах внутри окна
    // (диагностика further бисектится опцией NET_LOOP_MARK_EVERY_MS).
    dbgStepMark(12);
    LOGP("[BOOT] setup complete, survived to loop()\n");
}

// Старый netboot-задачный путь (0.6.2..0.6.5) оставлен как опция: сборка с
// -DNET_BOOT_USE_TASK=1 вернёт старт сети в отдельной задаче. По умолчанию
// с 0.6.6 сеть стартует синхронно в setup() (см. startNetworkStack()).
#ifndef NET_BOOT_USE_TASK
#define NET_BOOT_USE_TASK 0
#endif
#if NET_BOOT_USE_TASK
class NetBootTask : public os::TaskBase {
protected:
    void run() override { startNetworkStack(); }
};
#endif

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
#if NET_BOOT_USE_TASK
static NetBootTask netBootTask;    // опция: Wi-Fi + веб-сервер в задаче (стек 8 КБ)
#endif
static WdtMonitorTask wdtMonTask;  // диагностика зависаний по имени задачи

// 0.6.8: задача-обёртка для синхронного старта сети (см. startNetworkStack).
// Запускается из setup() на ядре 1, с приоритетом ВЫШЕ loopTask — иначе
// lowPriorityLoop (приоритет 1) блокирует loopTask до конца setup(), и
// xSemaphoreTake(loopDoneSem) внутри WiFi.setCore()/mode() не разблокируется
// никогда (ложная "зависимость" на шаге радио). Стек — динамический, 8 КБ.
static void netSetupTask(void *) {
    startNetworkStack();
    vTaskDelete(nullptr);
}

// ---- Канал RS485: UART1 + RE/DE (конфигурация — rs485_config.h) ----------
// FW 0.6.7: вынесено из середины setup() в отдельную функцию и вызывается
// ПОСЛЕ старта Wi-Fi/веб-сервера (RS485_INIT_AFTER_WIFI=1). Диагноз boot loop
// 0.6.2..0.6.6: крах ВСЕГДА случался в окне между "[MCP] online" и стартом
// радио, т.е. внутри hal::rsInit() — driver/uart-аллокации + привязка UART1
// к IO1/IO3 (пины Console-UART ROM-загрузчика). Сторонняя прошивка без
// RS485-драйвера поднимает AP безупречно — ранняя инициализация UART была
// единственной лишней операцией. Теперь радио стартует «чистым» запуском,
// а RS485 инициализируется после сети и при неудаче не блокирует систему.
// Прошивка пишет логи в Serial/UART0 (0.6.5+), драйвер RS485 сидит на UART1 —
// двойного владения портом нет. Мост логов в шину: UART_BRIDGE_TO_RS485=1.
static void initRs485Channel() {
    if (hal::rsInit()) {
#if RS485_START_MODBUS
        hal::rsSetMode(RsMode::MODBUS_RTU);
#endif
        hal::uartBridgeInit(UART_BRIDGE_TO_RS485 != 0);
        dbgStepMark(3);
        hal::logPrint("[RS485] ready @%u baud, mode=%s\n",
                      (unsigned)rs485_config::BAUD_RATE,
                      hal::rsMode() == RsMode::MODBUS_RTU ? "MODBUS_RTU" : "SIMPLE_UART");
    } else {
        // Драйвер НЕ захватил порт — Serial остаётся единственным каналом
        // логов; система продолжает работу без RS485 (non-blocking).
        hal::uartBridgeInit(false);
        dbgStepMark(3);
        hal::logPrint("[WARN] RS485 driver init failed (channel disabled)\n");
    }
    dbgStepMark(4);
}

// Причина последней перезагрузки (0.6.3): при boot loop без этой строки
// невозможно отличить крах прошивки от сбоя питания. esp_reset_reason()
// возвращает код причины (RST_REASON_*: PANIC / INT_WDT / TASK_WDT /
// BROWNOUT / SW_RESET / POWERON ...), расшифровка — в таблице ниже.
static const char *resetReasonName(int r) {
    switch (r) {
        case 1:  return "POWERON (включение/просадка питания)";
        case 3:  return "SW_RESET (программный restart)";
        case 4:  return "EXT (кнопка RST / внешний сигнал)";
        case 5:  return "UNKNOWN_RESET";
        case 6:  return "DBG_RESET (отладочный сброс)";
        case 7:  return "BRIDGE_RESET (USB-JTAG bridge)";
        case 8:  return "RTC_WATCHDOG (RTC WDT)";
        case 9:  return "PANIC/ABORT (крах прошивки!)";
        case 10: return "INT_WATCHDOG (INT WDT ядра)";
        case 11: return "TASK_WATCHDOG (TWDT зависшей задачи!)";
        case 12: return "BROWNOUT (просадка напряжения питания!)";
        default: return "other";
    }
}

// ---- [DBG-STEP] Маркер шагов инициализации (FW 0.6.4, boot-loop bisect) ----
// Сырая NVS-переменная (один u8) в namespace "koteldbg": запись дешёвая и
// не зависит от LittleFS/stdio/Serial, поэтому метка доживает до самого
// момента краха. Читаем её на каждом boot — получаем точную границу:
// шаги <= last step выполнены, крах — между last step и следующим маркером.
// Коды шагов: 1=pwr, 2=mcp, 3=rs485, 4=bridge, 5=littlefs, 6=tasks started,
// 9=netboot start, 7=wifi begin DONE, 8=web up DONE. -1 (0xFF) = метки нет.
// 0.6.5: маркер 9 пишется ПЕРЕД g_wifi.begin() — если при boot loop
// last step=9, крах происходит внутри инициализации Wi-Fi-радио.
static bool dbgNvsInit() {
    // nvs_flash_init идемпотентен; ESP_ERR_NVS_NEW_VERSION_FOUND /
    // NO_FREE_PAGES — стандартный путь Arduino: стирать и повторить.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    return err == ESP_OK;
}

static void dbgStepMark(uint8_t step) {
    static bool nvsReady = dbgNvsInit();
    if (!nvsReady) return;
    nvs_handle_t h;
    if (nvs_open("koteldbg", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "step", step);
        nvs_commit(h);
        nvs_close(h);
    }
}

static int dbgStepRead() {
    if (!dbgNvsInit()) return -1;
    nvs_handle_t h;
    uint8_t v = 0xFF;
    if (nvs_open("koteldbg", NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_u8(h, "step", &v) != ESP_OK) v = 0xFF;
        nvs_close(h);
    }
    return (v == 0xFF) ? -1 : (int)v;
}

static const char *dbgStepName(int s) {
    switch (s) {
        case -1: return "нет метки (первый запуск / стёртый NVS)";
        case 1:  return "pwr on";
        case 2:  return "mcp init";
        case 3:  return "rs485 init";
        case 4:  return "uart bridge";
        case 5:  return "littlefs mount";
        case 6:  return "tasks started";
        case 7:  return "wifi begin OK";
        case 8:  return "queued for web start (0.6.11; legacy: web up OK)";
        case 9:  return "tcpip core primed (0.6.11; legacy: netboot START)";
        case 10: return "Wi-Fi RADIO START (крах внутри WiFi.mode/softAP!)";
        case 11: return "softAP done (legacy)";
        case 12: return "setup complete, survived to loop (0.6.13)";
        case 13: return "loop() alive (0.6.13, NET_LOOP_MARK_EVERY_MS)";
        default: return "?";
    }
}

static void logBootResetReason() {
    const int r = (int)esp_reset_reason();
    LOGP("[BOOT] reset reason: %d = %s\n", r, resetReasonName(r));
    LOGP("[BOOT] heap=%u, maxblock=%u\n",
         (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
    // 0.6.4: последний завершённый шаг инициализации ПРЕДЫДУЩего запуска.
    // Если boot loop остаётся — по этому коду сразу видно, на каком шаге
    // происходит крах (см. таблицу шагов у dbgStepMark()).
    LOGP("[BOOT] last step=%d (%s)\n", dbgStepRead(), dbgStepName(dbgStepRead()));
}

// 0.6.8: диагностика раскладки задач по ядрам (штатная раскладка
// arduino-esp32 3.x задаётся на этапе сборки: loopTask/AsyncTCP/lwIP — core 1,
// esp_wifi — core 0; рантайм-API смены аффинности в этом SDK недоступен).
// Дамп uxTaskGetSystemState показывает фактическое ядро каждой задачи — по
// нему видно, что сетевой контекст не смешан с прикладным на одном ядре.
static void dumpTaskCoreAffinity() {
    static TaskStatus_t st[24];
    UBaseType_t n = uxTaskGetSystemState(st, 24, nullptr);
    for (UBaseType_t i = 0; i < n; ++i) {
        LOGP("[CORE] task \"%s\": core=%d, prio=%u\n",
             st[i].pcTaskName, (int)st[i].xCoreID, (unsigned)st[i].uxCurrentPriority);
    }
}

void setup() {
    const uint32_t bootMs = millis(); // бюджет времени инициализации MCP (non-blocking)
    // Отладочный Serial (до инициализации моста логи идут сюда же)
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);

    LOGP("\nKOTELPC boot. FW version: %s\n", FW_VERSION);
    logBootResetReason(); // 0.6.3: журнал причин перезагрузки (диагностика boot loop)
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

    // [DBG-STEP] Маркер последнего успешного шага инициализации (FW 0.6.4).
    // Пишется в NVS СЫРЫМ u8 (nvs_set_u8, без littlefs/stdio) — предельно
    // безопасно и доживает до момента краха. Если boot loop повторяется,
    // строка "[BOOT] last step=<код>" на следующем старте точно укажет шаг,
    // который НЕ завершился: 1=pwr, 2=mcp, 3=rs485, 4=bridge, 5=littlefs,
    // 6=tasks started, 7=netboot wifi, 8=netboot web up. -1 = метки нет.
    dbgStepMark(1);

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
    dbgStepMark(2);

    // 1c. LittleFS (раздел "storage" из partitions.csv)
    if (!fs::FsDriver::instance().begin(true)) {
        LOGP("[WARN] LittleFS mount failed\n");
    } else {
        fs::FsDriver::instance().mkdir("/log");
        fs::FsDriver::instance().mkdir("/www");   // каталог файлов веб-интерфейса
    }
    dbgStepMark(5);

    // 1d. Wi-Fi и веб-сервер (0.6.7): старт СИНХРОННО в setup() — ДО канала
    //     RS485. Порядок 0.6.2..0.6.6 (rsInit -> littlefs -> wifi) давал
    //     гарантированный boot loop: крах происходил сразу после [MCP] online,
    //     т.е. ВНУТРИ hal::rsInit() (driver/uart-аллокации + привязка UART1 к
    //     IO1/IO3 — пинам Console-UART ROM-загрузчика). Сторонняя прошивка без
    //     RS485-драйвера работает с AP идеально — значит проблема в ранней
    //     инициализации UART, а не в радио. Радио теперь поднимается «чистым»
    //     стартом, как в рабочей прошивке; маркер 10 пишется ПЕРЕД включением
    //     радио — при крахе внутри WiFi.mode()/softAP() в логе будет ровно
    //     last step=10. Опция возврата к задачному пути: -DNET_BOOT_USE_TASK=1.
#if !RS485_INIT_AFTER_WIFI
    // A/B-эксперимент: старый порядок — сначала RS485, потом сеть.
    initRs485Channel();
#endif
#if NET_BOOT_USE_TASK
    netBootTask.start(os::TaskBase::Config{
        "netboot", NET_BOOT_TASK_STACK_BYTES, NET_BOOT_TASK_PRIORITY, tskNO_AFFINITY});
#else
    // 0.6.10: СИНХРОННЫЙ вызов прямо из setup() — ровно как в эталонной
    // прошивке пользователя, где AP работает без единого сбоя. Выделенная
    // задача netsetup на core 1 (0.6.8/0.6.9) убрана: она стартовала сразу
    // после создания, ещё ДО запуска прикладных задач в конце setup(), и
    // конкурировала с ними за NVS/CPU именно в тот момент, когда поднималось
    // радио — это единственный содержательный difference с эталоном, оставшийся
    // в startup-пути. Опция возврата: -DNET_BOOT_USE_TASK=1.
    startNetworkStack();
#endif
#if RS485_INIT_AFTER_WIFI
    // 1e. Канал RS485: UART1 + RE/DE (конфигурация — rs485_config.h).
    //     ИНИЦИАЛИЗИРУЕТСЯ ПОСЛЕ Wi-Fi (см. комментарий выше): драйвер
    //     поднимается на UART1 (НЕ UART0!) и при неудаче не блокирует систему.
    //     Прошивка (0.6.5+) пишет логи в Serial/UART0; RS485 сидит на своих
    //     IO1/IO3 через UART1 — двойного владения портом больше нет.
    //     Управление мостом логов: UART_BRIDGE_TO_RS485 (project_config.h):
    //       0 = логи остаются в Serial (по умолчанию, безопасно: UART0 свободен);
    //       1 = логи/ввод гоняются по шине RS485 (для удалённой диагностики).
    initRs485Channel();
#endif

    // 3. Прикладные FreeRTOS-задачи
    heartbeatTask.start(os::TaskBase::Config{"hb", 3072, tskIDLE_PRIORITY + 1, 0});
    appLogicTask.start(os::TaskBase::Config{"app", 3072, tskIDLE_PRIORITY + 2, 0});
    fsLogTask.start(os::TaskBase::Config{"fslog", 4096, tskIDLE_PRIORITY + 1, 0});
    rs485Task.start(os::TaskBase::Config{"rs485", 4096, tskIDLE_PRIORITY + 2, 0});

    // 4. Монитор Watchdog Timer: подписывает задачи на esp_task_wdt и при
    //    просрочке heartbeat позволяет ядру напечатать имя зависшей задачи.
    wdtMonTask.start(os::TaskBase::Config{"wdtmon", 3072, tskIDLE_PRIORITY + 1, 0});

    dbgStepMark(6);
    LOGP("[MAIN] tasks started\n");

    // 4b. Веб-сервер (0.6.11): старт ПОСЛЕ прикладных задач и дампа ядер.
    // В 0.6.10 s_server.begin() вызывался из setup() сразу после softAP —
    // в момент, когда tcpip_thread ещё не успевал зарегистрироваться владельцем
    // блокировки ядра TCP/IP (см. startNetworkStack). Теперь радио-стек
    // «прогрет» (netif поднят, tcpip_thread живёт), loopTask уже работает —
    // listen-портоваться безопасно. Функция идемпотентна (s_started).
    web::start();

    // 5. 0.6.8: диагностика раскладки по ядрам — штатно loopTask/AsyncTCP/
    //    lwIP живут на core 1, esp_wifi на core 0; прикладные задачи закреплены
    //    за core 0 (см. выше). Дамп [CORE] позволяет это проверить в логе.
    dumpTaskCoreAffinity();
}

void loop() {
    // В Arduino-каркасе loop() сам по себе является задачей FreeRTOS.
    // Вся полезная работа вынесена в задачи выше.
    // FW 0.6.8: здесь же обрабатываются СЕТЕВЫЕ запросы из веб-UI
    // (переподключение Wi-Fi). Все вызовы esp_wifi/lwIP обязаны идти из
    // контекста loopTask (ядро Arduino): сторонние задачи, дёргающие
    // WiFi.begin()/mode(), ломали блокировки ядра TCP/IP — крах
    // "assert failed: tcp_alloc ... Required to lock TCPIP core".
    // FW 0.6.12: диспатч WiFi-событий выполняется автоматически ядром
    // esp_event (_arduino_event_cb -> обработчик WiFi.onEvent); метод
    // WiFi.run() в Arduino-core 3.x отсутствует — отдельный вызов из
    // loopTask не требуется, событие ARDUINO_EVENT_WIFI_AP_START (и все
    // последующие STA/AP-события) устанавливает s_tcpipReady сам.
    web::handleLoopTick();
#if NET_LOOP_MARK_EVERY_MS
    // FW 0.6.13 диагностика: периодический NVS-маркер из loopTask — если
    // сброс приходит уже в loop(), на следующем boot будет last step=13.
    static uint32_t s_lastLoopMark = 0;
    if (millis() - s_lastLoopMark >= NET_LOOP_MARK_EVERY_MS) {
        s_lastLoopMark = millis();
        dbgStepMark(13);
    }
#endif
    vTaskDelay(pdMS_TO_TICKS(10));
}
