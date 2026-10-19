// Драйвер GPIO: реализация hal::gpio* поверх ESP-IDF driver/gpio.
// Конфигурация пинов берётся ИСКЛЮЧИТЕЛЬНО из gpio_config.h.

#include "driver_gpio.h"

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>

static const char *DRV_TAG = "GPIO_DRV";

namespace drv {

GpioDriver &GpioDriver::instance() {
    static GpioDriver inst;
    return inst;
}

bool GpioDriver::isValid(GpioPin pin) const {
    return static_cast<int>(pin) >= 0 && static_cast<int>(pin) < gpio_config::kCount;
}

bool GpioDriver::init() {
    for (int i = 0; i < gpio_config::kCount; ++i) {
        const auto &cfg = gpio_config::table[i];
        gpio_config_t io = {};
        io.pin = cfg.pin;
        const bool isInput =
            cfg.mode == HalGpioMode::INPUT ||
            cfg.mode == HalGpioMode::INPUT_PULLUP ||
            cfg.mode == HalGpioMode::INPUT_PULLDOWN;
        io.mode = isInput ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT;
        io.pull_up_en = (cfg.mode == HalGpioMode::INPUT_PULLUP) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
        io.pull_down_en = (cfg.mode == HalGpioMode::INPUT_PULLDOWN) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
        io.intr_type = GPIO_INTR_DISABLE;

        esp_err_t err = gpio_config(&io);
        if (err != ESP_OK) {
            ESP_LOGE(DRV_TAG, "pin %d config failed: %s", static_cast<int>(cfg.pin), esp_err_to_name(err));
            return false;
        }

        if (!isInput) {
            // Начальное состояние с учётом инверсии
            const bool level = cfg.initialHigh ? !cfg.activeLow : cfg.activeLow;
            gpio_set_level(cfg.pin, level ? 1 : 0);
        }
        ESP_LOGI(DRV_TAG, "pin %2d [%d] ready (%s)", static_cast<int>(cfg.pin), i,
                 isInput ? "input" : "output");
    }
    initialized_ = true;
    return true;
}

bool GpioDriver::write(GpioPin pin, bool active) {
    if (!initialized_ || !isValid(pin)) return false;
    const auto &cfg = gpio_config::table[static_cast<int>(pin)];
    const bool level = cfg.activeLow ? !active : active;
    return gpio_set_level(cfg.pin, level ? 1 : 0) == ESP_OK;
}

bool GpioDriver::read(GpioPin pin) const {
    if (!initialized_ || !isValid(pin)) return false;
    const auto &cfg = gpio_config::table[static_cast<int>(pin)];
    const int level = gpio_get_level(cfg.pin);
    return cfg.activeLow ? (level == 0) : (level == 1);
}

void GpioDriver::setLevel(GpioPin pin, bool high) {
    if (!initialized_ || !isValid(pin)) return;
    gpio_set_level(gpio_config::table[static_cast<int>(pin)].pin, high ? 1 : 0);
}

bool GpioDriver::getLevel(GpioPin pin) const {
    if (!initialized_ || !isValid(pin)) return false;
    return gpio_get_level(gpio_config::table[static_cast<int>(pin)].pin) == 1;
}

} // namespace drv

// ---- Реализация HAL ----
namespace hal {

bool gpioInit() { return drv::GpioDriver::instance().init(); }

bool gpioWrite(int logicalPin, bool active) {
    return drv::GpioDriver::instance().write(static_cast<GpioPin>(logicalPin), active);
}

bool gpioRead(int logicalPin) {
    return drv::GpioDriver::instance().read(static_cast<GpioPin>(logicalPin));
}

void gpioSetLevel(int logicalPin, bool high) {
    drv::GpioDriver::instance().setLevel(static_cast<GpioPin>(logicalPin), high);
}

bool gpioGetLevel(int logicalPin) {
    return drv::GpioDriver::instance().getLevel(static_cast<GpioPin>(logicalPin));
}

} // namespace hal
