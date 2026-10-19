#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

#include <stdint.h>
#include <driver/gpio.h>
#include "hal/hal_gpio.h"

// ============================================================================
// Конфигурация GPIO — единственный источник истины по пинам и режимам.
// Все изменения назначений пинов делаются ТОЛЬКО здесь (в коде драйвера).
// Остальной код работает только с логическими именами (GpioPin) через HAL.
// ============================================================================

// Логические имена сигналов (не зависят от физических номеров пинов)
enum class GpioPin : uint8_t {
    LED_STATUS,     // индикатор состояния
    RELAY_1,        // реле 1 (насос / котёл)
    RELAY_2,        // реле 2 (доп. нагрузка)
    BTN_1,          // кнопка 1
    SENSOR_1,       // вход датчика 1 (например, DS18B20 / сухой контакт)
    PIN_COUNT
};

struct GpioSignalConfig {
    gpio_num_t pin;      // физический пин ESP32
    HalGpioMode mode;    // INPUT / OUTPUT / INPUT_PULLUP / INPUT_PULLDOWN
    bool activeLow;      // true: сигнал активен при низком уровне
    bool initialHigh;    // начальное состояние выхода (учитывает activeLow)
};

namespace kGpioConfig {

constexpr int kCount = static_cast<int>(GpioPin::PIN_COUNT);

// Таблица конфигурации. Индекс = порядковый номер GpioPin.
// Пины 34..39 — input-only (подтяжки недоступны на 34/35).
constexpr GpioSignalConfig table[kCount] = {
    /* LED_STATUS */ { GPIO_NUM_2,  HalGpioMode::OUTPUT_MODE,       false, false },
    /* RELAY_1    */ { GPIO_NUM_26, HalGpioMode::OUTPUT_MODE,       false, false },
    /* RELAY_2    */ { GPIO_NUM_27, HalGpioMode::OUTPUT_MODE,       false, false },
    /* BTN_1      */ { GPIO_NUM_33, HalGpioMode::INPUT_PULLUP_MODE, true,  false },
    /* SENSOR_1   */ { GPIO_NUM_32, HalGpioMode::INPUT_PULLUP_MODE, false, false },
};

constexpr bool validate() {
    for (int i = 0; i < kCount; ++i) {
        const auto &c = table[i];
        const int p = static_cast<int>(c.pin);
        if (p < 0 || p > 39) return false;
        const bool isInput =
            c.mode == HalGpioMode::INPUT_MODE ||
            c.mode == HalGpioMode::INPUT_PULLUP_MODE ||
            c.mode == HalGpioMode::INPUT_PULLDOWN_MODE;
        // выходы недоступны на input-only пинах 34..39
        if (!isInput && p >= 34 && p <= 39) return false;
        // strapped-пины 6..11 не используем как выходы нагрузки
        if (!isInput && p >= 6 && p <= 11) return false;
    }
    return true;
}

static_assert(validate(), "Invalid GPIO configuration in gpio_config.h");

} // namespace kGpioConfig

#endif // GPIO_CONFIG_H
