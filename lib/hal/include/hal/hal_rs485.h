#ifndef HAL_RS485_H
#define HAL_RS485_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// ============================================================================
// HAL последовательного канала RS485 (UART + управление RE/DE на IO23).
// Прикладной код не знает ни пинов, ни регистров UART: конфигурация живёт
// в драйвере (lib/drivers/include/rs485_config.h), сюда приходят только
// логические операции. Потокобезопасно (мьютекс внутри драйвера).
//
// Режимы работы выбираются НА ЛЕТУ:
//   RsMode::SIMPLE_UART  — «простой UART»: read/write как потоковый порт,
//                          без кадров и таймаутов (отладочный протокол,
//                          текстовые команды, эхо-режим и т.п.).
//   RsMode::MODBUS_RTU   — кадры: запись кадра целиком с автоматическим
//                          переключением RE/DE, чтение по межсимвольному
//                          таймауту t3.5 + проверка CRC16-Modbus.
// ============================================================================

enum class RsMode : uint8_t {
    SIMPLE_UART = 0,
    MODBUS_RTU = 1,
};

namespace hal {

bool rsInit();                       // инициализация UART + пина RE/DE (до создания задач)
void rsDeinit();

RsMode rsMode();                     // текущий режим
bool rsSetMode(RsMode mode);         // переключение режима на лету (потокобезопасно)

int  rsAvailable();                  // сколько байт доступно для чтения
size_t rsRead(uint8_t *buf, size_t len, uint32_t timeoutMs);  // низкоуровневое чтение
size_t rsWrite(const uint8_t *data, size_t len);              // низкоуровневая запись (TX-only)

// Кадровый API (используется в обоих режимах; в MODBUS_RTU добавляется CRC/t3.5):
// Отправить кадр: data..data+len-1. В режиме MODBUS_RTU CRC16 дописывается
// автоматически (len>=2, buffer должен вмещать +2 байта).
bool rsSendFrame(const uint8_t *data, size_t len);
// Принять кадр: ожидание до timeoutMs, завершение по t3.5 (MODBUS_RTU)
// или по наличию данных (SIMPLE_UART). В MODBUS_RTU проверяется CRC16.
size_t rsReceiveFrame(uint8_t *buf, size_t maxLen, uint32_t timeoutMs);

// Прозрачный вывод потока в Serial (для отладки шины)
void rsEnableEcho(bool enable);

} // namespace hal

#endif // HAL_RS485_H
