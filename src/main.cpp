// KOTELPC — точка входа. FreeRTOS-задачи + LittleFS + драйверы GPIO через HAL.
//
// Карта железа (конфигурация — только в коде драйвера):
//   IO21 = SDA MCP23017, IO22 = SCL MCP23017  (mcp23017_config.h)
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

// Прототипы вспомогательных функций уровня приложения (определения ниже)
static void simpleUartPrint(const char *msg);

// ---- Задача heartbeat: пульсация светодиода HEART на IO16 (через HAL/драйвер) ----
class HeartbeatTask : public os::TaskBase {
protected:
    void run() override {
        while (isRunning()) {
            state_ = !state_;
            hal::gpioWrite(static_cast<int>(GpioPin::LED_HEART), state_);
            delayMs(HEARTBEAT_PERIOD_MS);
        }
    }
private:
    bool state_ = false;
};

// ---- Задача опроса дискретных входов DI0..DI7 (MCP23017 порт B, дебаунс в драйвере) ----
class DiScanTask : public os::TaskBase {
protected:
    void run() override {
        uint8_t lastActive = 0x00;
        while (isRunning()) {
            // Дебаунс выполняется внутри драйвера одним пакетным чтением порта B
            hal::mcpPollInputs();

            // Собираем слово активных входов DI0..DI7
            uint8_t active = 0;
            for (int i = 0; i < 8; ++i) {
                if (hal::mcpDiActive(static_cast<int>(static_cast<uint8_t>(McpPin::DI0) + i))) {
                    active |= (1 << i);
                }
            }
            if (active != lastActive) {
                LOGP("[DI] active mask = 0x%02X", active);
                for (int i = 0; i < 8; ++i) {
                    const bool on = (active >> i) & 1;
                    if (on != ((lastActive >> i) & 1)) {
                        LOGP("  DI%d=%d", i, on ? 1 : 0);
                    }
                }
                LOGP("\n");
                lastActive = active;
            }
            delayMs(DI_POLL_INTERVAL_MS);
        }
    }
};

// ---- Задача теста реле: ОДНОКРАТНО по очереди щёлкает всеми реле RL0..RL7 ----
// Частота переключения — RELAY_TEST_FREQ_HZ (2 Гц => полупериод 250 мс на реле).
// По завершении все реле выключаются, в простой UART выдаётся
// сообщение "Тест реле пройден", после чего задача завершается сама.
class RelayTestTask : public os::TaskBase {
protected:
    void run() override {
        // Задержка перед стартом: даём завершиться инициализации железа
        delayMs(RELAY_TEST_START_DELAY_MS);
        LOGP("[RTEST] старт: RL0..RL7 по очереди, %d Гц\n", RELAY_TEST_FREQ_HZ);
        bool ok = true;
        for (int i = 0; i < 8; ++i) {
            // Включаем текущее реле (RELAY_ACTIVE_LOW учитывается драйвером)
            if (!hal::mcpRelaySet(static_cast<int>(static_cast<uint8_t>(McpPin::RL0) + i), true)) {
                ok = false;
                LOGP("[RTEST] RL%d: ошибка записи (MCP23017 недоступен)\n", i);
                break;
            }
            delayMs(RELAY_TEST_HALFPERIOD_MS);
            hal::mcpRelaySet(static_cast<int>(static_cast<uint8_t>(McpPin::RL0) + i), false);
            delayMs(RELAY_TEST_HALFPERIOD_MS);
            LOGP("[RTEST] RL%d проверено\n", i);
        }
        // Гарантированно выключаем все реле
        for (int i = 0; i < 8; ++i) {
            hal::mcpRelaySet(static_cast<int>(static_cast<uint8_t>(McpPin::RL0) + i), false);
        }
        if (ok) {
            // Сообщение о выполнении — в простой UART (IO1/IO3, тот же UART0)
            simpleUartPrint("Тест реле пройден\r\n");
            LOGP("[RTEST] завершён успешно\n");
        } else {
            simpleUartPrint("Тест реле: ОШИБКА (см. лог)\r\n");
            LOGP("[RTEST] завершён с ошибкой\n");
        }
        // Однократная задача: trampoline сам завершит её по выходе из run()
    }
};

// ---- Задача FS: проверка смонтированного LittleFS, запись лога ----
class FsLogTask : public os::TaskBase {
protected:
    void run() override {
        uint32_t n = 0;
        while (isRunning()) {
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

static HeartbeatTask heartbeatTask;
static DiScanTask diScanTask;
#if RELAY_TEST_ENABLED
static RelayTestTask relayTestTask;   // однократный тест реле при старте
#endif
static FsLogTask fsLogTask;
static Rs485Task rs485Task;
static volatile bool rs485Ready_ = false;  // UART0 перехвачен RS485-драйвером

// Простой UART (IO1/IO3): до инициализации RS485-драйвера это обычный Serial,
// после — тот же UART0, проброшенный в Rs485Driver. Пишем прямо в порт, чтобы
// сообщения теста уходили в простой UART независимо от режима моста логов.
static void simpleUartPrint(const char *msg) {
    if (rs485Ready_) {
        const size_t len = strlen(msg);
        hal::rsWrite((const uint8_t *)msg, len);
    } else {
        Serial.print(msg);
    }
}

void setup() {
    // Отладочный Serial (до инициализации моста логи идут сюда же)
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);

    LOGP("\nKOTELPC boot. FW version: %s\n", FW_VERSION);
    LOGP("Chip: %s, rev %d, cores %d, freq %d MHz\n",
         ESP.getChipModel(), ESP.getChipRevision(),
         ESP.getChipCores(), ESP.getCpuFreqMHz());
    LOGP("Pins: IO16=HEART LED, IO1=RS485 TX, IO3=RS485 RX, IO23=RE/DE, "
         "IO21=SDA, IO22=SCL -> MCP23017 (GPA=RL0..RL7, GPB=DI0..DI7)\n");

    // 1. Драйверы железа (конфигурация пинов — внутри драйвера)
    if (!hal::gpioInit()) {
        LOGP("[FATAL] GPIO driver init failed\n");
    }

    // 1b. Расширитель GPIO MCP23017 по I2C (шина/адрес/порты — в mcp23017_config.h)
    if (hal::mcpInit()) {
        LOGP("[MCP] MCP23017 online @0x20 (sda=21 scl=22)\n");
    } else {
        LOGP("[WARN] MCP23017 not present on I2C bus\n");
    }

    // 1c. Канал RS485: UART + RE/DE (конфигурация — rs485_config.h).
    //     Arduino HWSerial (Serial) использует UART0 — освобождаем порт
    //     перед установкой driver/uart поверх тех же пинов.
    Serial.end();
    if (hal::rsInit()) {
        rs485Ready_ = true;   // UART0 теперь обслуживает Rs485Driver
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
    }

    // 3. Прикладные FreeRTOS-задачи
    heartbeatTask.start(os::TaskBase::Config{"hb", 3072, tskIDLE_PRIORITY + 1, tskNO_AFFINITY});
    diScanTask.start(os::TaskBase::Config{"di", 3072, tskIDLE_PRIORITY + 2, tskNO_AFFINITY});
#if RELAY_TEST_ENABLED
    // Однократный тест реле: старт с задержкой 2 с (железо инициализировано),
    // после завершения задача сама удаляет себя из планировщика
    relayTestTask.start(os::TaskBase::Config{"rtest", 3072, tskIDLE_PRIORITY + 1, tskNO_AFFINITY});
#endif
    fsLogTask.start(os::TaskBase::Config{"fslog", 4096, tskIDLE_PRIORITY + 1, tskNO_AFFINITY});
    rs485Task.start(os::TaskBase::Config{"rs485", 4096, tskIDLE_PRIORITY + 2, tskNO_AFFINITY});

    LOGP("[MAIN] tasks started\n");
}

void loop() {
    // В Arduino-каркасе loop() сам по себе является задачей FreeRTOS.
    // Вся полезная работа вынесена в задачи выше.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
