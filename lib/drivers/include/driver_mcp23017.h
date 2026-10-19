#ifndef DRIVER_MCP23017_H
#define DRIVER_MCP23017_H

#include <stdint.h>
#include <stdbool.h>
#include "mcp23017_config.h"

// ============================================================================
// Драйвер расширителя GPIO MCP23017 (I2C).
// Вся конфигурация (шина, адрес, режимы портов) — в mcp23017_config.h.
// Потокобезопасен (мьютекс на обращения к Wire), может вызываться из задач.
// ============================================================================

namespace drv {

class Mcp23017Driver {
public:
    static Mcp23017Driver &instance();

    bool init();                       // Wire.begin + проверка чипа + настройка портов
    bool isReady() const { return ready_; }

    bool write(McpPin pin, bool high); // установить уровень пина (read-modify-write OLATx)
    bool read(McpPin pin) const;       // прочитать уровень пина (GPIOx)

    bool writePort(uint8_t port, uint8_t value);      // port: 0=A, 1=B ( latch OLATx)
    bool readPort(uint8_t port, uint8_t &value);      // port: 0=A, 1=B (GPIOx)

    static bool isValid(McpPin pin);

private:
    Mcp23017Driver() = default;
    bool ready_ = false;
    void *lock_ = nullptr; // SemaphoreHandle_t (не раскрываем FreeRTOS в заголовке)

    bool ensureLock();
    void releaseLock();
    bool writeReg(uint8_t addr, uint8_t value);
    bool readReg(uint8_t addr, uint8_t &value);
    static uint8_t portOf(McpPin pin);
    static uint8_t bitOf(McpPin pin);
};

} // namespace drv

#endif // DRIVER_MCP23017_H
