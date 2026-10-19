// KOTELPC — минимальная отладочная прошивка (LED ONLY).
//
// В main оставлено ТОЛЬКО мигание светодиода HEART на IO16: 1 с вкл / 1 с выкл.
// Светодиод активный-низкий (включение = лог.0 на пине) — полярность учитывает
// драйвер GPIO по флагу activeLow в gpio_config.h.
//
// Всё остальное (MCP23017, RS485/Modbus, LittleFS, FreeRTOS-задачи DI/реле/FS)
// временно отключено для отладки железа. Полный код — в истории git
// (коммиты до "debug: leave only LED blink in main").

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "project_config.h"
#include "hal/hal_gpio.h"
#include "hal/hal_uart_bridge.h"
#include "driver_gpio.h"

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);

    LOGP("\nKOTELPC boot (LED-only debug). FW version: %s\n", FW_VERSION);

    // Инициализация драйвера GPIO (конфигурация пинов — внутри драйвера).
    if (!hal::gpioInit()) {
        LOGP("[FATAL] GPIO driver init failed\n");
    }

    // Стартовое состояние: светодиод выключен.
    hal::gpioWrite(static_cast<int>(GpioPin::LED_HEART), false);
}

void loop() {
    // Мигание LED HEART: 1 сек вкл, 1 сек выкл (скважность 50%).
    static bool on = false;
    on = !on;
    hal::gpioWrite(static_cast<int>(GpioPin::LED_HEART), on);
    vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_HALF_PERIOD_MS));
}
