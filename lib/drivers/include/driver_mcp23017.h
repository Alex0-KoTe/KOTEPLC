#ifndef DRIVER_MCP23017_H
#define DRIVER_MCP23017_H

#include <stdint.h>
#include <stdbool.h>
#include "mcp23017_config.h"

// ============================================================================
// Драйвер расширителя GPIO MCP23017 (I2C).
// Вся конфигурация (шина, адрес, режимы портов) — в mcp23017_config.h.
// Потокобезопасен (мьютекс на обращениях к Wire), может вызываться из задач.
// Дискретные входы DI0..DI7 (порт B): diActive() выполняет живое чтение
// порта; активный уровень входа задаётся ACTIVE_LOW_INPUT.
// ============================================================================

namespace drv {

class Mcp23017Driver {
public:
    static Mcp23017Driver &instance();

    bool init();                       // I2C-мастер + проверка чипа + настройка портов

    // Wire (NG-драйвер Arduino) — единственный I2C мастер проекта.
    // Нативный ESP-IDF мастер использовать нельзя: Wire занимает порты
    // 0 и 1 одновременно, его создание вызывает фатальный abort
    // "CONFLICT! driver_ng is not allowed to be used with this old driver".
    static void ensureWireBus();       // идемпотентный Wire.begin + clock/timeout

    bool isReady() const { return ready_; }

    bool write(McpPin pin, bool high); // установить прямой уровень пина (read-modify-write OLATx)
    bool read(McpPin pin) const;       // прочитать прямой уровень пина (GPIOx)

    // Реле (порт A): семантика "активно" с учётом RELAY_ACTIVE_LOW
    bool relaySet(McpPin pin, bool active);
    // Дискретные входы (порт B): стабилизированное дебаунсом состояние
    bool diActive(McpPin pin) const;

    bool writePort(uint8_t port, uint8_t value); // port: 0=A, 1=B (latch OLATx)
    bool readPort(uint8_t port, uint8_t &value); // port: 0=A, 1=B (GPIOx)

    // Дебаунс порта B (входы DI0..DI7). Вызывать периодически из задачи опроса.
    void pollInputs();                   // пакетное чтение GPIOB + машина дебаунса
    uint8_t debouncedState() const;      // стабилизированное слово DI0..DI7 (бит i = DIi активен)

    static bool isValid(McpPin pin);

private:
    Mcp23017Driver() = default;
    bool ready_ = false;
    static bool sWireReady_;  // Wire.begin() выполнен (мастер проекта)
    bool nativeReady_ = false; // нативный мастер отключён (конфликт с Wire/NG)
    void *lock_ = nullptr; // SemaphoreHandle_t (не раскрываем FreeRTOS в заголовке)

    // Дебаунс порта B: сырое значение, стабилизированное значение, время изменения
    uint8_t rawB_ = 0xFF;
    uint8_t stableB_ = 0xFF;
    uint32_t changeTick_[8] = {0};

    bool ensureLock();
    void releaseLock();
    bool writeReg(uint8_t addr, uint8_t value);
    bool readReg(uint8_t addr, uint8_t &value);
    bool writeRegQuiet(uint8_t addr, uint8_t value); // без вывода ошибки в лог
    bool readRegQuiet(uint8_t addr, uint8_t &value); // без вывода ошибки в лог
    static bool probeBus(uint8_t addr);   // ACK-опрос адреса на шине (true = ACK)
    static int  probeBusDetailed(uint8_t addr); // код endTransmission(): 0=ACK, 2=NACK, 4=timeout
    static bool sdaStuckLow();            // проверка линии SDA через INPUT_PULLUP
    static void recoverBus();             // 9 тактов SCL + STOP (I2C bus recovery)
    static uint8_t portOf(McpPin pin);
    static uint8_t bitOf(McpPin pin);
};

} // namespace drv

#endif // DRIVER_MCP23017_H
