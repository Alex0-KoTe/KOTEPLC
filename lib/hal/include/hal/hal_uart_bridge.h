#ifndef HAL_UART_BRIDGE_H
#define HAL_UART_BRIDGE_H

#include <stdint.h>
#include <stddef.h>

// ============================================================================
// Развязка отладочного вывода/ввода и промышленного канала RS485.
// Реализация — в lib/drivers/src/uart_bridge.cpp: задачи печатают через
// hal::logPrint()/hal::inputPoll(), а мост сам решает, куда идут потоки:
//
//   DEBUG-канал (по умолчанию): Serial (отладочный порт, как раньше)
//   RS485-канал              : драйвер RS485 (UART + RE/DE на IO23)
//
// Переключение возможно на лету из любой задачи (потокобезопасно).
// В RS485-режиме логи уходят в шину, а команды читаются с шины.
// ============================================================================

namespace hal {

bool uartBridgeInit(bool startInRs485Mode); // вызывается после rsInit()
void uartToRs485(bool enable);              // true: логи/ввод <-> RS485, false: <-> Serial
bool uartIsRs485();

// Порты для прикладного кода (printf-семантика; ввод неблокирующий)
void logPrint(const char *fmt, ...);
size_t inputPoll(uint8_t *buf, size_t maxLen); // >0: прочитано, 0: нет данных

} // namespace hal

// Удобный макрос вместо прямых вызовов Serial в задачах
#define LOGP(...) hal::logPrint(__VA_ARGS__)

#endif // HAL_UART_BRIDGE_H
