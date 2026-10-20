#ifndef DRIVER_RS485_H
#define DRIVER_RS485_H

#include <stdint.h>
#include <stddef.h>
#include "rs485_config.h"
#include "hal/hal_rs485.h"

// ============================================================================
// Драйвер RS485-канала (UART + RE/DE). Реализует hal::rs* интерфейс.
// Конфигурация пинов и UART — ИСКЛЮЧИТЕЛЬНО в rs485_config.h.
// Потокобезопасен: все операции под мьютексом FreeRTOS; переключение
// SIMPLE_UART <-> MODBUS_RTU возможно на лету из любой задачи.
// ============================================================================

namespace drv {

class Rs485Driver {
public:
    static Rs485Driver &instance();

    bool init();
    void deinit();

    RsMode mode() const { return mode_; }
    bool setMode(RsMode m);

    int  available();
    size_t read(uint8_t *buf, size_t len, uint32_t timeoutMs);
    size_t write(const uint8_t *data, size_t len);

    bool sendFrame(const uint8_t *data, size_t len);
    size_t receiveFrame(uint8_t *buf, size_t maxLen, uint32_t timeoutMs);

    void enableEcho(bool enable) { echo_ = enable; }

    // CRC16-Modbus (полином 0xA001, init 0xFFFF)
    static uint16_t crc16(const uint8_t *data, size_t len);

private:
    Rs485Driver() = default;
    bool inited_ = false;
    bool ready_ = false;
    RsMode mode_ = RsMode::SIMPLE_UART;
    bool echo_ = false;
    void *lock_ = nullptr; // SemaphoreHandle_t

    bool ensureLock();
    void releaseLock();
    void setTxMode(bool tx);          // переключить RE/DE
    size_t flushRxStale(size_t keep); // сбросить «грязный» приём
};

} // namespace drv

#endif // DRIVER_RS485_H
