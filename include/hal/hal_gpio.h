#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// HAL GPIO — абстракция доступа к пинам.
// Прикладной код не знает физических номеров пинов: он использует только
// логические имена GpioPin. Конфигурация пин-аутов находится в драйвере
// (lib/drivers/include/gpio_config.h).
// ============================================================================

enum class HalGpioMode : uint8_t {
    INPUT,
    OUTPUT,
    INPUT_PULLUP,
    INPUT_PULLDOWN,
};

namespace hal {

// Инициализировать все пины по таблице gpio_config (вызывается драйвером).
bool gpioInit();

// Запись/чтение с учётом инверсии activeLow ("активен" == true).
bool gpioWrite(int logicalPin, bool active);
bool gpioRead(int logicalPin);           // true = сигнал активен

// Прямой доступ к физическому уровню (для диагностики).
void gpioSetLevel(int logicalPin, bool high);
bool gpioGetLevel(int logicalPin);

} // namespace hal

#endif // HAL_GPIO_H
