// KOTELPC — точка входа. FreeRTOS-задачи + LittleFS + драйверы GPIO через HAL.
//
// Карта железа (конфигурация — только в коде драйвера):
//   IO21 = SDA MCP23017, IO22 = SCL MCP23017  (mcp23017_config.h)
//   IO16 = HEART pulse LED                     (gpio_config.h)
//   IO1  = UART0 TX, IO3 = UART0 RX            (Serial, 115200)
//   MCP23017: GPA0..GPA7 = реле RL0..RL7, GPB0..GPB7 = входы DI0..DI7 (дебаунс)

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "project_config.h"
#include "hal/hal_gpio.h"
#include "hal/hal_mcp23017.h"
#include "driver_gpio.h"
#include "mcp23017_config.h"
#include "fs_driver.h"
#include "freertos_task.h"

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
                Serial.printf("[DI] active mask = 0x%02X", active);
                for (int i = 0; i < 8; ++i) {
                    const bool on = (active >> i) & 1;
                    if (on != ((lastActive >> i) & 1)) {
                        Serial.printf("  DI%d=%d", i, on ? 1 : 0);
                    }
                }
                Serial.println();
                lastActive = active;
            }
            delayMs(DI_POLL_INTERVAL_MS);
        }
    }
};

// ---- Задача реле: тестовая «бегущая» последовательность RL0..RL7 (порт A) ----
// В рабочей прошивке этот паттерн заменяется логикой управления котлом.
class RelayTask : public os::TaskBase {
protected:
    void run() override {
        uint8_t pattern = 0x01;
        while (isRunning()) {
            // Пакетная запись порта A: бит i = состояние реле RL_i
            // (RELAY_ACTIVE_LOW учитывается драйвером при поштучном mcpRelaySet;
            //  при пакетной записи уровень задаётся напрямую)
            hal::mcpWritePortA(pattern);
            pattern = (uint8_t)((pattern << 1) | (pattern >> 7));
            delayMs(RELAY_TEST_INTERVAL_MS);
        }
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
                    Serial.printf("[FS] %zu/%zu bytes used\n", used, total);
                }
            }
            delayMs(FS_LOG_INTERVAL_MS);
        }
    }
};

static HeartbeatTask heartbeatTask;
static DiScanTask diScanTask;
static RelayTask relayTask;
static FsLogTask fsLogTask;

void setup() {
    // UART0: IO1 = TX, IO3 = RX (назначение пинов — в ядре Arduino ESP32)
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);

    Serial.println();
    Serial.printf("KOTELPC boot. FW version: %s\n", FW_VERSION);
    Serial.printf("Chip: %s, rev %d, cores %d, freq %d MHz\n",
                  ESP.getChipModel(), ESP.getChipRevision(),
                  ESP.getChipCores(), ESP.getCpuFreqMHz());
    Serial.println("Pins: IO16=HEART LED, IO1=UART TX, IO3=UART RX, "
                   "IO21=SDA, IO22=SCL -> MCP23017 (GPA=RL0..RL7, GPB=DI0..DI7)");

    // 1. Драйверы железа (конфигурация пинов — внутри драйвера)
    if (!hal::gpioInit()) {
        Serial.println("[FATAL] GPIO driver init failed");
    }

    // 1b. Расширитель GPIO MCP23017 по I2C (шина/адрес/порты — в mcp23017_config.h)
    if (hal::mcpInit()) {
        // Тестовый цикл: включить/выключить RL0 — проверка линии и реле
        hal::mcpRelaySet(static_cast<int>(McpPin::RL0), true);
        delay(100);
        hal::mcpRelaySet(static_cast<int>(McpPin::RL0), false);
        Serial.println("[MCP] MCP23017 online @0x20 (sda=21 scl=22)");
    } else {
        Serial.println("[WARN] MCP23017 not present on I2C bus");
    }

    // 2. LittleFS (раздел "storage" из partitions.csv)
    if (!fs::FsDriver::instance().begin(true)) {
        Serial.println("[WARN] LittleFS mount failed");
    } else {
        fs::FsDriver::instance().mkdir("/log");
    }

    // 3. Прикладные FreeRTOS-задачи
    heartbeatTask.start(os::TaskBase::Config{"hb", 3072, tskIDLE_PRIORITY + 1, tskNO_AFFINITY});
    diScanTask.start(os::TaskBase::Config{"di", 3072, tskIDLE_PRIORITY + 2, tskNO_AFFINITY});
    relayTask.start(os::TaskBase::Config{"relay", 3072, tskIDLE_PRIORITY + 1, tskNO_AFFINITY});
    fsLogTask.start(os::TaskBase::Config{"fslog", 4096, tskIDLE_PRIORITY + 1, tskNO_AFFINITY});

    Serial.println("[MAIN] tasks started");
}

void loop() {
    // В Arduino-каркасе loop() сам по себе является задачей FreeRTOS.
    // Вся полезная работа вынесена в задачи выше.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
