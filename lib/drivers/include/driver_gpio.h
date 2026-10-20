#ifndef DRIVER_GPIO_H
#define DRIVER_GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include "hal/hal_gpio.h"
#include "gpio_config.h"

// ============================================================================
// Драйвер GPIO: реализует HAL поверх ESP-IDF driver/gpio.
// Вся конфигурация пинов — в gpio_config.h (код драйвера).
// Потокобезопасен (атомарные операции GPIO_WRITE_MASK registers),
// может вызываться из задач FreeRTOS.
// ============================================================================

namespace drv {

class GpioDriver {
public:
    static GpioDriver &instance();

    bool init();
    bool write(GpioPin pin, bool active);   // "active=true" -> сигнал активен
    bool read(GpioPin pin) const;           // true = сигнал активен
    void setLevel(GpioPin pin, bool high);  // прямой уровень
    bool getLevel(GpioPin pin) const;
    bool isValid(GpioPin pin) const;

private:
    GpioDriver() = default;
    bool initialized_ = false;
};

} // namespace drv

#endif // DRIVER_GPIO_H
