#ifndef RS485_CONFIG_H
#define RS485_CONFIG_H

#include <stdint.h>
#include "driver/uart.h"

// ============================================================================
// Конфигурация транслятора RS485 на UART ESP32 — единственный источник истины.
// Железо KOTELPC:
//   IO1  = UART1 TX  -> DI (вход данных передатчика RS485-транслятора)
//   IO3  = UART1 RX  <- RO (выход приёмника RS485-транслятора)
//   IO23 = RE/DE     -> направление передачи (HIGH = передаём, LOW = принимаем)
// Вся конфигурация GPIO lives здесь, в коде драйвера; прикладной код
// работает только через hal::rs* интерфейс.
//
// !!! ВАЖНО (boot-loop fix FW 0.6.5): порт — UART_NUM_1, НЕ UART_NUM_0 !!!
// IO1/IO3 физически подключены к транслятору RS485, но это ОНИ ЖЕ пины
// отладочного UART0, который Arduino core использует для Serial (лог +
// прошивка). В 0.6.x драйвер захватывал UART0 и перепривязывал его на
// IO1/IO3 — это ломало внутренний объект Serial (двойное владение портом),
// после чего запуск Wi-Fi (netboot step 7) утыкался в испорченный UART-стек
// и плата уходила в boot loop («rst:0xC SW_CPU_RESET» сразу после строки
// [MCP] MCP23017 online). Теперь Serial остаётся единственным владельцем
// UART0, а RS485 получает UART1 на тех же пинах. Единственное ограничение:
// во время flash-прошивки линиями IO1/IO3 управляет ROM-загрузчик — штатные
// модули RS485 это tolerate (приёмник не мешает загрузчику).
// ============================================================================

namespace rs485_config {

constexpr uart_port_t UART_PORT = UART_NUM_1; // отдельный контроллер: Serial не трогает UART0 (см. шапку)
constexpr int PIN_TX = 1;                     // IO1  -> DI транслятора
constexpr int PIN_RX = 3;                     // IO3  <- RO транслятора
constexpr int PIN_REDE = 23;                  // IO23 -> RE/DE транслятора

constexpr uint32_t BAUD_RATE = 19200;         // типовой Modbus RTU 8N1
constexpr uart_word_length_t DATA_BITS = UART_DATA_8_BITS;
constexpr uart_parity_t PARITY = UART_PARITY_DISABLE;
constexpr uart_stop_bits_t STOP_BITS = UART_STOP_BITS_1;

constexpr size_t RX_BUF_SIZE = 1024;          // буфер приёма driver/uart
constexpr size_t TX_BUF_SIZE = 512;           // буфер передачи driver/uart

// Активный уровень RE/DE: HIGH = передача (типовые MAX485/SP3485).
// Для инверсных трансляторов поставьте false.
constexpr bool REDE_ACTIVE_HIGH = true;

// Guard-time после последних байт перед переходом в приём, мкс.
// Покрывает задержку чипов трансивера и хвост последнего стоп-бита.
constexpr uint32_t TX_GUARD_US = 200;

// Межсимвольный таймаут Modbus RTU (t3.5) вычисляется из скорости:
// 3.5 символа * 10 бит / baud. Минимум 2 мс (требование спецификации).
constexpr uint32_t modbusT35Ms(uint32_t baud) {
    uint32_t t = (baud == 0) ? 10 : (uint32_t)((7ULL * 1000000ULL) / (10ULL * baud) + 1);
    return t < 2 ? 2 : t;
}

} // namespace rs485_config

#endif // RS485_CONFIG_H
