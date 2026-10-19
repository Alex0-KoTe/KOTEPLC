// Мост «отладочные потоки <-> RS485-канал» для KOTELPC.
// Конфигурация пинов/режимов UART живёт в драйвере (rs485_config.h),
// здесь только коммутация: logPrint/инпут идут либо в Serial (DEBUG),
// либо в UART0 через Rs485Driver (RS485). Оба направления переключаются
// на лету; операции сериализуются мьютексом.

#include "hal/hal_uart_bridge.h"
#include "driver_rs485.h"

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static void *s_lock = nullptr;
static bool s_rs485 = false;

static bool takeLock() {
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    return s_lock && xSemaphoreTake((SemaphoreHandle_t)s_lock, pdMS_TO_TICKS(100)) == pdTRUE;
}
static void giveLock() { if (s_lock) xSemaphoreGive((SemaphoreHandle_t)s_lock); }

namespace hal {

bool uartBridgeInit(bool startInRs485Mode) {
    s_rs485 = false;
    uartToRs485(startInRs485Mode);
    return true;
}

void uartToRs485(bool enable) {
    if (!takeLock()) return;
    if (s_rs485 != enable) {
        s_rs485 = enable;
        ESP_LOGI("UARTBR", "log/input channel -> %s", enable ? "RS485 bus" : "Serial (debug)");
    }
    giveLock();
}

bool uartIsRs485() { return s_rs485; }

void logPrint(const char *fmt, ...) {
    char line[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    if (!takeLock()) return;
    const bool toRs = s_rs485;
    giveLock();

    if (toRs) {
        drv::Rs485Driver::instance().write((const uint8_t *)line, strlen(line));
    } else {
        Serial.print(line);
    }
}

size_t inputPoll(uint8_t *buf, size_t maxLen) {
    if (!buf || maxLen == 0) return 0;
    if (!takeLock()) return 0;
    const bool fromRs = s_rs485;
    giveLock();

    if (fromRs) {
        // Неблокирующее чтение: сколько байт накопилось — столько забираем
        int avail = drv::Rs485Driver::instance().available();
        if (avail <= 0) return 0;
        size_t want = (size_t)avail > maxLen ? maxLen : (size_t)avail;
        return drv::Rs485Driver::instance().read(buf, want, 0);
    }
    size_t got = 0;
    while (got < maxLen && Serial.available() > 0) {
        buf[got++] = (uint8_t)Serial.read();
    }
    return got;
}

} // namespace hal
