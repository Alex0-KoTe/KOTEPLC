#ifndef HAL_MCP23017_H
#define HAL_MCP23017_H

#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// HAL расширителя GPIO MCP23017.
// Прикладной код не знает ни адресов I2C, ни регистров, ни номеров пинов:
// конфигурация живёт в драйвере (lib/drivers/include/mcp23017_config.h),
// а сюда приходит только логический номер пина (значение enum McpPin).
// Потокобезопасно: доступ к шине сериализуется мьютексом внутри драйвера.
//
// Семантика "active":
//   mcpWrite(pin, true)  -> реле включено (учитывает RELAY_ACTIVE_LOW)
//   mcpReadActive(di)    -> дебаунсенное состояние входа DI (true = активен)
// ============================================================================

namespace hal {

bool mcpInit();                          // инициализация шины + конфигурация портов
bool mcpIsReady();                       // true, если чип отвечает на I2C
void mcpForceInitFallback();             // сброс IOCON (Bank0/slew-off) после неудачной инициализации

bool mcpWrite(int logicalPin, bool high);    // прямой уровень пина (без инверсии)
bool mcpRead(int logicalPin);                // прямой уровень пина (сырое чтение)
bool mcpRelaySet(int logicalPin, bool active); // вкл/выкл реле с учётом ACTIVE_LOW
bool mcpDiActive(int logicalPin);            // дебаунсенное состояние входа DI

bool mcpWritePortA(uint8_t value);           // пакетная запись порта A (реле RL0..RL7)
bool mcpWritePortB(uint8_t value);           // пакетная запись порта B
bool mcpReadPortA(uint8_t &value);           // пакетное чтение порта A
bool mcpReadPortB(uint8_t &value);           // пакетное чтение порта B (DI0..DI7, сырые)

void mcpPollInputs();                        // периодический опрос+дебаунс входов (задача вызывает ~каждые 5-10 мс)

} // namespace hal

#endif // HAL_MCP23017_H
