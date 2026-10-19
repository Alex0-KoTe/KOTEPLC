#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

#include <stdint.h>
#include <driver/gpio.h>
#include "hal/hal_gpio.h"

// ============================================================================
// Конфигурация GPIO ESP32 — единственный источник истины по пинам и режимам.
// Все изменения назначений пинов делаются ТОЛЬКО здесь (в коде драйвера).
// Остальной код работает только с логическими именами (GpioPin) через HAL.
//
// Карта железа KOTELPC:
//   IO21 -> SDA  шины I2C к MCP23017 (назначается в mcp23017_config.h)
//   IO22 -> SCL  шины I2C к MCP23017 (назначается в mcp23017_config.h)
//   IO16 -> HEART pulse LED (включается подачей LOGIC 0 — activeLow)
//   IO4  -> PERIPH_EN: логическая 1 включает питание всей периферии
//           (MCP23017, реле, RS485) после старта программы
//   IO1  -> UART TX (DI транслятора RS485; конфигурация — rs485_config.h)
//   IO3  -> UART RX (RO транслятора RS485; загрубочный пин — осторожно)
//   IO23 -> RE/DE направления передачи RS485 (конфигурация — rs485_config.h)
// ============================================================================

// Логические имена сигналов (не зависят от физических номеров пинов)
enum class GpioPin : uint8_t {
    LED_HEART,        // индикатор «сердцебиения» (LED горит при LOW на IO16)
    PERIPH_EN,        // разрешение питания периферии (LOGIC 1 = включено)
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
// Реле (RL0..RL7) и дискретные входы (DI0..DI7) висят на MCP23017
// и описаны в mcp23017_config.h, здесь их нет.
constexpr GpioSignalConfig table[kCount] = {
    /* LED_HEART  */ { GPIO_NUM_16, HalGpioMode::OUTPUT_MODE, true,  false }, // LED горит при LOW (activeLow)
    /* PERIPH_EN  */ { GPIO_NUM_4,  HalGpioMode::OUTPUT_MODE, false, false }, // старт: периферия обесточена, включим после gpioInit()
};

// Пины, занятые под другие функции (I2C/UART) — не должны попадать в таблицу.
constexpr int kReservedPins[] = {
    21, // SDA  (MCP23017, mcp_config::SDA_PIN)
    22, // SCL  (MCP23017, mcp_config::SCL_PIN)
    1,  // UART TX  (RS485, rs485_config::PIN_TX)
    3,  // UART RX  (RS485, rs485_config::PIN_RX)
    23, // RE/DE    (RS485, rs485_config::PIN_REDE)
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
        // запрещаем дубли зарезервированных пинов (I2C/UART)
        for (int r = 0; r < (int)(sizeof(kReservedPins) / sizeof(kReservedPins[0])); ++r) {
            if (p == kReservedPins[r]) return false;
        }
    }
    return true;
}

static_assert(validate(), "Invalid GPIO configuration in gpio_config.h");

} // namespace kGpioConfig

#endif // GPIO_CONFIG_H
