#ifndef MCP23017_CONFIG_H
#define MCP23017_CONFIG_H

#include <stdint.h>

// ============================================================================
// Конфигурация расширителя GPIO MCP23017 (I2C) — единственный источник истины.
// Все изменения шины, адреса и назначения пинов A0..A7/B0..B7 делаются
// ТОЛЬКО здесь (в коде драйвера). Прикладной код работает с логическими
// именами McpPin через hal::mcpRead()/hal::mcpWrite().
// ============================================================================

namespace mcp_config {

// ---- Шина I2C (ESP32 Wire) ----
constexpr uint8_t  SDA_PIN   = 21;   // физический пин ESP32 SDA
constexpr uint8_t  SCL_PIN   = 22;   // физический пин ESP32 SCL
constexpr uint32_t CLOCK_HZ  = 100000; // 100 кГц (стандартный режим I2C)

// ---- Адрес микросхемы ----
// 0x20..0x27 в зависимости от уровней ADDR (A2 A1 A0).
// На плате ADDR подтянут к GND -> 0x20.
constexpr uint8_t I2C_ADDR = 0x20;

// ---- Режимы портов (битовые маски, бит0 = pin0 ... бит7 = pin7) ----
// 1 = вход, 0 = выход.
constexpr uint8_t IODIRA = 0xFF; // порт A: все пины входами (группа входов PLC)
constexpr uint8_t IODIRB = 0x00; // порт B: все пинами выходами (группа выходов PLC)

// Подтяжки (GPPU): 1 = internal pull-up включён (только для входов)
constexpr uint8_t GPPUA  = 0xFF;
constexpr uint8_t GPPUB  = 0x00;

// Инверсия уровня (GPIOINV): 1 = инвертировать чтение/запись порта
constexpr uint8_t GPIOINVA = 0x00;
constexpr uint8_t GPIOINVB = 0x00;

} // namespace mcp_config

// Логические имена сигналов расширителя.
// Старший бит (0x10) кодирует номер порта: 0 = порт A, 1 = порт B.
enum class McpPin : uint8_t {
    // Порт A — входы (например, дискретные датчики / концевики)
    IN_1 = 0x00,
    IN_2 = 0x01,
    IN_3 = 0x02,
    IN_4 = 0x03,
    IN_5 = 0x04,
    IN_6 = 0x05,
    IN_7 = 0x06,
    IN_8 = 0x07,
    // Порт B — выходы (например, реле / исполнительные устройства)
    OUT_1 = 0x10,
    OUT_2 = 0x11,
    OUT_3 = 0x12,
    OUT_4 = 0x13,
    OUT_5 = 0x14,
    OUT_6 = 0x15,
    OUT_7 = 0x16,
    OUT_8 = 0x17,
    PIN_COUNT = 16,
};

static_assert(static_cast<int>(McpPin::PIN_COUNT) == 16,
              "MCP23017 has exactly 16 pins");

#endif // MCP23017_CONFIG_H
