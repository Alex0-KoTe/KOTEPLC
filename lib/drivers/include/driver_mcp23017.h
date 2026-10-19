#ifndef DRIVER_MCP23017_H
#define DRIVER_MCP23017_H

#include <stdint.h>
#include <stdbool.h>
#include "mcp23017_config.h"

// ============================================================================
// Драйвер расширителя GPIO MCP23017 (I2C).
// Вся конфигурация (шина, адрес, режимы портов) — в mcp23017_config.h.
// Потокобезопасен (мьютекс на обращениях к Wire), может вызываться из задач.
// Встроенный программный дебаунс дискретных входов DI0..DI7 (порт B):
// pollInputs() опрашивает порт одним чтением и обновляет стабилизированные
// состояния; активный уровень входа задаётся ACTIVE_LOW_INPUT.
// ============================================================================

namespace drv {

class Mcp23017Driver {
public:
    static Mcp23017Driver &instance();

    bool init();                       // Wire.begin + проверка чипа + настройка портов
    bool isReady() const { return ready_; }

    bool write(McpPin pin, bool high); // установить прямой уровень пина (read-modify-write OLATx)
    bool read(McpPin pin) const;       // прочитать прямой уровень пина (GPIOx)

    // Реле (порт A): семантика "активно" с учётом RELAY_ACTIVE_LOW
    bool relaySet(McpPin pin, bool active);
    // Дискретные входы (порт B): стабилизированное дебаунсом состояние
    bool diActive(McpPin pin) const;

    bool writePort(uint8_t port, uint8_t value); // port: 0=A, 1=B (latch OLATx)
    bool readPort(uint8_t port, uint8_t &value); // port: 0=A, 1=B (GPIOx)

    void pollInputs();                 // опрос порта B + дебаунс (вызывать из задачи ~5-10 мс)

    static bool isValid(McpPin pin);

    // [DIAG-PULLUP] сырой доступ к регистрам для самодиагностики шины:
    // вызывающий обязан держать мьютекс (mcpDiagLock/Unlock). Адреса регистров
    // — Bank0: GPPUA=0x0C, GPPUB=0x0D, GPIOB=0x13, OLATA=0x14.
    bool diagWriteReg(uint8_t addr, uint8_t value) { return writeReg(addr, value); }
    bool diagReadReg(uint8_t addr, uint8_t &value) { return readReg(addr, value); }
    bool diagEnsureLock()   { return ensureLock(); }
    void diagReleaseLock()  { releaseLock(); }

private:
    Mcp23017Driver() = default;
    bool ready_ = false;
    void *lock_ = nullptr; // SemaphoreHandle_t (не раскрываем FreeRTOS в заголовке)

    // Дебаунс порта B: сырое значение, стабилизированное значение, время изменения
    uint8_t rawB_ = 0xFF;
    uint8_t stableB_ = 0xFF;
    uint32_t changeTick_[8] = {0};

    bool ensureLock();
    void releaseLock();
    bool writeReg(uint8_t addr, uint8_t value);
    bool readReg(uint8_t addr, uint8_t &value);
    static bool probeBus(uint8_t addr);   // ACK-опрос адреса на шине (true = ACK)
    static int  probeBusDetailed(uint8_t addr); // код endTransmission(): 0=ACK, 2=NACK, 4=timeout
    static bool sdaStuckLow();            // проверка линии SDA через INPUT_PULLUP
    static void recoverBus();             // 9 тактов SCL + STOP (I2C bus recovery)
    static uint8_t portOf(McpPin pin);
    static uint8_t bitOf(McpPin pin);
};

} // namespace drv

#endif // DRIVER_MCP23017_H
