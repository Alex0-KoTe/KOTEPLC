// KOTELPC — точка входа. FreeRTOS-задачи + LittleFS + драйверы GPIO через HAL.
// Конфигурация пинов — только в lib/drivers/include/gpio_config.h.

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "project_config.h"
#include "hal/hal_gpio.h"
#include "drivers/driver_gpio.h"
#include "fs/fs_driver.h"
#include "freertos_task.h"

// ---- Задача heartbeat: мигание статусным светодиодом (через HAL/драйвер) ----
class HeartbeatTask : public os::TaskBase {
protected:
    void run() override {
        while (isRunning()) {
            state_ = !state_;
            hal::gpioWrite(static_cast<int>(GpioPin::LED_STATUS), state_);
            delayMs(HEARTBEAT_PERIOD_MS);
        }
    }
private:
    bool state_ = false;
};

// ---- Задача мониторинха кнопок/входов (дебаунс программный) ----
class InputTask : public os::TaskBase {
protected:
    void run() override {
        bool last = false;
        uint32_t changeAt = 0;
        while (isRunning()) {
            const bool raw = hal::gpioRead(static_cast<int>(GpioPin::BTN_1));
            const uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (raw != last) {
                if (changeAt == 0) changeAt = now;
                else if (now - changeAt >= BUTTON_DEBOUNCE_MS) {
                    last = raw;
                    changeAt = 0;
                    Serial.printf("[IN] BTN_1 = %d\n", last ? 1 : 0);
                }
            } else {
                changeAt = 0;
            }
            delayMs(INPUT_POLL_INTERVAL_MS);
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
            snprintf(line, sizeof(line), "[%lu] fs alive, heap=%u\n",
                     (unsigned long)(xTaskGetTickCount() / 1000),
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
static InputTask inputTask;
static FsLogTask fsLogTask;

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);

    Serial.println();
    Serial.printf("KOTELPC boot. FW version: %s\n", FW_VERSION);
    Serial.printf("Chip: %s, rev %d, cores %d, freq %d MHz\n",
                  ESP.getChipModel(), ESP.getChipRevision(),
                  ESP.getChipCores(), ESP.getCpuFreqMHz());
    Serial.printf("FreeRTOS: tick=%u Hz, idle hooks on\n",
                  (unsigned)configTICK_RATE_HZ);

    // 1. Драйверы железа (конфигурация пинов — внутри драйвера)
    if (!hal::gpioInit()) {
        Serial.println("[FATAL] GPIO driver init failed");
    }

    // 2. LittleFS (раздел "storage" из partitions.csv)
    if (!fs::FsDriver::instance().begin(true)) {
        Serial.println("[WARN] LittleFS mount failed");
    } else {
        fs::FsDriver::instance().mkdir("/log");
    }

    // 3. Прикладные FreeRTOS-задачи
    heartbeatTask.start({"hb",   3072, tskIDLE_PRIORITY + 1, tskNO_AFFINITY});
    inputTask.start({    "input", 3072, tskIDLE_PRIORITY + 2, tskNO_AFFINITY});
    fsLogTask.start({    "fslog", 4096, tskIDLE_PRIORITY + 1, tskNO_AFFINITY});

    Serial.println("[MAIN] tasks started");
}

void loop() {
    // В Arduino-каркасе loop() сам по себе является задачей FreeRTOS.
    // Вся полезная работа вынесена в задачи выше.
    vTaskDelay(pdMS_TO_TICKS(1000));
}

