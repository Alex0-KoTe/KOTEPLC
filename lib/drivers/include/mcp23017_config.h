#ifndef MCP23017_CONFIG_H
#define MCP23017_CONFIG_H

#include <stdint.h>

// ============================================================================
// Конфигурация расширителя GPIO MCP23017 (I2C) — единственный источник истины.
// Все изменения шины, адреса и назначения пинов A0..A7/B0..B7 делаются
// ТОЛЬКО здесь (в коде драйвера). Прикладной код работает с логическими
// именами McpPin через hal::mcpRead()/hal::mcpWrite().
//
// Карта железа KOTELPC:
//   IO21 = SDA, IO22 = SCL  (шина I2C к MCP23017)
//   GPA0..GPA7 -> релейные выходы RL0..RL7
//   GPB0..GPB7 -> дискретные входы DI0..DI7 (с программным дебаунсом)
// ============================================================================

namespace mcp_config {

// ---- Шина I2C (ESP32 Wire) ----
// DIAG-SWAP: предположение — на плате SDA/SCL перепутаны (напрямую с GPIO),
// поэтому пины инвертированы: тактование идёт на IO21, данные — на IO22.
// Если диагностика подтвердит работу чипа — оставить как есть и обновить
// документацию; если нет — вернуть 21/22.
constexpr uint8_t  SDA_PIN   = 22;     // IO22 -> фактическая линия данных (SDA)
constexpr uint8_t  SCL_PIN   = 21;     // IO21 -> фактическая линия тактов (SCL)
// 50 кГц — устойчивый минимум для длинных/шумных линий и медленных фронтов
// после включения питания через PERIPH_EN. Если диагностика показывает
// стабильный ACK, можно поднять до 100000 (а затем и до 400000).
constexpr uint32_t CLOCK_HZ  = 50000;
// Таймаут транзакции Wire в мс (по умолчанию 500 мс — слишком много для опроса).
constexpr uint16_t WIRE_TIMEOUT_MS = 100;

// ---- Адрес микросхемы ----
// 0x20..0x27 в зависимости от уровней ADDR (A2 A1 A0).
// На плате ADDR подтянут к GND -> 0x20.
constexpr uint8_t I2C_ADDR = 0x20;

// ---- Режимы портов (битовые маски, бит0 = pin0 ... бит7 = pin7) ----
// 1 = вход, 0 = выход.
constexpr uint8_t IODIRA = 0x00; // порт A: выходы — реле RL0..RL7
constexpr uint8_t IODIRB = 0xFF; // порт B: входы — дискретные DI0..DI7

// Подтяжки (GPPU): 1 = internal pull-up включён (только для входов)
// DI0..DI7 — «сухой контакт» к GND: держим внутренние подтяжки включёнными,
// замкнутый контакт = низкий уровень (см. ACTIVE_LOW_INPUT ниже).
constexpr uint8_t GPPUA  = 0x00;
constexpr uint8_t GPPUB  = 0xFF;

// Инверсия уровня (GPIOINV/IPOL): 1 = инвертировать чтение/запись порта
constexpr uint8_t GPIOINVA = 0x00;
constexpr uint8_t GPIOINVB = 0x00;

// Активные состояния сигналов (учитываются драйвером на уровне HAL-семантики
// "active == true"):
//   Реле: активный уровень на выходе = HIGH (активен HIGH) — типовой модуль
//         реле с оптопарой IN->HIGH. Если у вас реле активны по LOW,
//         поставьте RELAY_ACTIVE_LOW = true.
//   Входы DI: закрытый контакт замыкает пин на GND => сигнал активен при LOW.
constexpr bool RELAY_ACTIVE_LOW  = false;
constexpr bool ACTIVE_LOW_INPUT  = true;

// Дебаунс дискретных входов (мс): вход считается стабилизированным, если
// сырое значение не менялось дольше этого времени.
constexpr uint16_t DEBOUNCE_MS = 30;

} // namespace mcp_config

// Логические имена сигналов расширителя.
// Старший бит (0x10) кодирует номер порта: 0 = порт A, 1 = порт B.
enum class McpPin : uint8_t {
    // Порт A — релейные выходы RL0..RL7 (GPA0..GPA7)
    RL0 = 0x00,
    RL1 = 0x01,
    RL2 = 0x02,
    RL3 = 0x03,
    RL4 = 0x04,
    RL5 = 0x05,
    RL6 = 0x06,
    RL7 = 0x07,
    // Порт B — дискретные входы DI0..DI7 (GPB0..GPB7, с дебаунсом)
    DI0 = 0x10,
    DI1 = 0x11,
    DI2 = 0x12,
    DI3 = 0x13,
    DI4 = 0x14,
    DI5 = 0x15,
    DI6 = 0x16,
    DI7 = 0x17,
    PIN_COUNT = 16,
};

// Алиасы старой схемы именования (IN_n -> DI, OUT_n -> RL), совместимость
enum class McpAlias : uint8_t {
    IN_1  = static_cast<uint8_t>(McpPin::DI0),
    IN_2  = static_cast<uint8_t>(McpPin::DI1),
    IN_3  = static_cast<uint8_t>(McpPin::DI2),
    IN_4  = static_cast<uint8_t>(McpPin::DI3),
    IN_5  = static_cast<uint8_t>(McpPin::DI4),
    IN_6  = static_cast<uint8_t>(McpPin::DI5),
    IN_7  = static_cast<uint8_t>(McpPin::DI6),
    IN_8  = static_cast<uint8_t>(McpPin::DI7),
    OUT_1 = static_cast<uint8_t>(McpPin::RL0),
    OUT_2 = static_cast<uint8_t>(McpPin::RL1),
    OUT_3 = static_cast<uint8_t>(McpPin::RL2),
    OUT_4 = static_cast<uint8_t>(McpPin::RL3),
    OUT_5 = static_cast<uint8_t>(McpPin::RL4),
    OUT_6 = static_cast<uint8_t>(McpPin::RL5),
    OUT_7 = static_cast<uint8_t>(McpPin::RL6),
    OUT_8 = static_cast<uint8_t>(McpPin::RL7),
};

static_assert(static_cast<int>(McpPin::PIN_COUNT) == 16,
              "MCP23017 has exactly 16 pins");

#endif // MCP23017_CONFIG_H
