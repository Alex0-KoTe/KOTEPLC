#ifndef HAL_MCP23017_H
#define HAL_MCP23017_H

#include <stdbool.h>

// ============================================================================
// HAL расширителя GPIO MCP23017.
// Прикладной код не знает ни адресов I2C, ни регистров, ни номеров пинов:
// конфигурация живёт в драйвере (lib/drivers/include/mcp23017_config.h),
// а сюда приходит только логический номер пина (значение enum McpPin).
// Потокобезопасно: доступ к шине сериализуется мьютексом внутри драйвера.
// ============================================================================

namespace hal {

bool mcpInit();                       // инициализация шины + конфигурация портов
bool mcpIsReady();                    // true, если чип отвечает на I2C

bool mcpWrite(int logicalPin, bool high); // установить уровень выхода
bool mcpRead(int logicalPin);             // прочитать уровень входа/выхода

bool mcpWritePortA(uint8_t value);        // пакетная запись порта A (OUTPUT latch)
bool mcpWritePortB(uint8_t value);        // пакетная запись порта B
bool mcpReadPortA(uint8_t &value);        // пакетное чтение порта A (GPIO register)
bool mcpReadPortB(uint8_t &value);        // пакетное чтение порта B

} // namespace hal

#endif // HAL_MCP23017_H
