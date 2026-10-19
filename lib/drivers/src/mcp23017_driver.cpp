// Драйвер расширителя GPIO MCP23017 (I2C, регистровая модель Bank0).
// Конфигурация шины, адреса и пинов — ИСКЛЮЧИТЕЛЬНО в mcp23017_config.h.
// Реализует hal::mcp* (см. lib/hal/include/hal/hal_mcp23017.h).
// Потокобезопасен: все обращения к шине под мьютексом FreeRTOS.

#include "driver_mcp23017.h"

#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <esp_log.h>

static const char *MCP_TAG = "MCP23017";

// ---- Регистры MCP23017 (Bank 0, CONFIG.BANK=0, дефолт после POR) ----
namespace reg {
constexpr uint8_t IODIRA  = 0x00;
constexpr uint8_t IODIRB  = 0x01;
constexpr uint8_t IPOLA   = 0x02;
constexpr uint8_t IPOLB   = 0x03;
constexpr uint8_t GPINTENA = 0x04;
constexpr uint8_t GPINTENB = 0x05;
constexpr uint8_t DEFVALA = 0x06;
constexpr uint8_t DEFVALB = 0x07;
constexpr uint8_t INTCONA = 0x08;
constexpr uint8_t INTCONB = 0x09;
constexpr uint8_t IOCONA  = 0x0A;
constexpr uint8_t IOCONB  = 0x0B;
constexpr uint8_t GPPUA   = 0x0C;
constexpr uint8_t GPPUB   = 0x0D;
constexpr uint8_t INTFA   = 0x0E;
constexpr uint8_t INTFB   = 0x0F;
constexpr uint8_t INTCAPA = 0x10;
constexpr uint8_t INTCAPB = 0x11;
constexpr uint8_t GPIOA   = 0x12;
constexpr uint8_t GPIOB   = 0x13;
constexpr uint8_t OLATA   = 0x14;
constexpr uint8_t OLATB   = 0x15;
} // namespace reg

namespace drv {

Mcp23017Driver &Mcp23017Driver::instance() {
    static Mcp23017Driver inst;
    return inst;
}

bool Mcp23017Driver::ensureLock() {
    if (!lock_) {
        lock_ = xSemaphoreCreateMutex();
    }
    return lock_ && xSemaphoreTake((SemaphoreHandle_t)lock_, pdMS_TO_TICKS(100)) == pdTRUE;
}

void Mcp23017Driver::releaseLock() {
    if (lock_) xSemaphoreGive((SemaphoreHandle_t)lock_);
}

bool Mcp23017Driver::writeReg(uint8_t addr, uint8_t value) {
    Wire.beginTransmission(mcp_config::I2C_ADDR);
    Wire.write(addr);
    Wire.write(value);
    const bool ok = Wire.endTransmission() == 0;
    if (!ok) {
        ESP_LOGE(MCP_TAG, "write reg 0x%02X failed", addr);
    }
    return ok;
}

bool Mcp23017Driver::readReg(uint8_t addr, uint8_t &value) {
    if (Wire.requestFrom((int)mcp_config::I2C_ADDR, (int)addr, 1) != 1) {
        ESP_LOGE(MCP_TAG, "read reg 0x%02X failed", addr);
        return false;
    }
    value = Wire.read();
    return true;
}

bool Mcp23017Driver::init() {
    if (!Wire.begin(mcp_config::SDA_PIN, mcp_config::SCL_PIN, mcp_config::CLOCK_HZ)) {
        ESP_LOGE(MCP_TAG, "I2C bus init failed (sda=%u scl=%u)",
                 mcp_config::SDA_PIN, mcp_config::SCL_PIN);
        return false;
    }

    // Проверка присутствия чипа: запись IOCON должна подтвердиться ACK'ом
    if (!ensureLock()) return false;
    const bool present = writeReg(reg::IOCONA, 0x00) && [&] {
        uint8_t v = 0xFF;
        return readReg(reg::IOCONA, v) && v == 0x00;
    }();
    releaseLock();

    if (!present) {
        ESP_LOGE(MCP_TAG, "chip not found at 0x%02X", mcp_config::I2C_ADDR);
        return false;
    }

    // Конфигурация портов из таблицы драйвера
    if (!ensureLock()) return false;
    bool ok = writeReg(reg::IODIRA, mcp_config::IODIRA) &&
              writeReg(reg::IODIRB, mcp_config::IODIRB) &&
              writeReg(reg::GPPUA,  mcp_config::GPPUA)  &&
              writeReg(reg::GPPUB,  mcp_config::GPPUB)  &&
              writeReg(reg::IPOLA,  0x00) &&
              writeReg(reg::IPOLB,  0x00) &&
              writeReg(reg::GPINTENA, 0x00) && // прерывания по изменению выкл.
              writeReg(reg::GPINTENB, 0x00) &&
              writeReg(reg::OLATA,  0x00) &&
              writeReg(reg::OLATB,  0x00);
    releaseLock();

    if (!ok) {
        ESP_LOGE(MCP_TAG, "port configuration failed");
        return false;
    }

    ready_ = true;
    ESP_LOGI(MCP_TAG, "ready @0x%02X (sda=%u scl=%u, %u Hz)",
             mcp_config::I2C_ADDR, mcp_config::SDA_PIN, mcp_config::SCL_PIN,
             (unsigned)mcp_config::CLOCK_HZ);
    return true;
}

bool Mcp23017Driver::isValid(McpPin pin) const {
    const auto v = static_cast<uint8_t>(pin);
    return v <= 0x07 || (v >= 0x10 && v <= 0x17);
}

uint8_t Mcp23017Driver::portOf(McpPin pin) { return (static_cast<uint8_t>(pin) >> 4) & 1; }
uint8_t Mcp23017Driver::bitOf(McpPin pin)  { return static_cast<uint8_t>(pin) & 0x07; }

bool Mcp23017Driver::write(McpPin pin, bool high) {
    if (!ready_ || !isValid(pin)) return false;
    const uint8_t port = portOf(pin);
    const uint8_t bit = bitOf(pin);
    const uint8_t olat = port ? reg::OLATB : reg::OLATA;

    if (!ensureLock()) return false;
    uint8_t cur = 0;
    bool ok = readReg(olat, cur);
    if (ok) {
        cur = high ? (cur | (1 << bit)) : (cur & ~(1 << bit));
        ok = writeReg(olat, cur);
    }
    releaseLock();
    return ok;
}

bool Mcp23017Driver::read(McpPin pin) const {
    if (!ready_ || !isValid(pin)) return false;
    const uint8_t port = portOf(pin);
    const uint8_t bit = bitOf(pin);
    const uint8_t gpio = port ? reg::GPIOB : reg::GPIOA;

    uint8_t val = 0;
    if (!const_cast<Mcp23017Driver *>(this)->ensureLock()) return false;
    const bool ok = const_cast<Mcp23017Driver *>(this)->readReg(gpio, val);
    const_cast<Mcp23017Driver *>(this)->releaseLock();
    if (!ok) return false;
    return (val >> bit) & 1;
}

bool Mcp23017Driver::writePort(uint8_t port, uint8_t value) {
    if (!ready_) return false;
    if (!ensureLock()) return false;
    const bool ok = writeReg(port ? reg::OLATB : reg::OLATA, value);
    releaseLock();
    return ok;
}

bool Mcp23017Driver::readPort(uint8_t port, uint8_t &value) {
    if (!ready_) return false;
    if (!ensureLock()) return false;
    const bool ok = readReg(port ? reg::GPIOB : reg::GPIOA, value);
    releaseLock();
    return ok;
}

} // namespace drv

// ---- Реализация HAL ----
namespace hal {

bool mcpInit() { return drv::Mcp23017Driver::instance().init(); }
bool mcpIsReady() { return drv::Mcp23017Driver::instance().isReady(); }

bool mcpWrite(int logicalPin, bool high) {
    return drv::Mcp23017Driver::instance().write(static_cast<McpPin>(logicalPin), high);
}

bool mcpRead(int logicalPin) {
    return drv::Mcp23017Driver::instance().read(static_cast<McpPin>(logicalPin));
}

bool mcpWritePortA(uint8_t value) { return drv::Mcp23017Driver::instance().writePort(0, value); }
bool mcpWritePortB(uint8_t value) { return drv::Mcp23017Driver::instance().writePort(1, value); }
bool mcpReadPortA(uint8_t &value) { return drv::Mcp23017Driver::instance().readPort(0, value); }
bool mcpReadPortB(uint8_t &value) { return drv::Mcp23017Driver::instance().readPort(1, value); }

} // namespace hal
