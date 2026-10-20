// Драйвер канала RS485: UART (driver/uart) + управление направлением RE/DE.
// Вся конфигурация пинов/скрости — ИСКЛЮЧИТЕЛЬНО в rs485_config.h.
// Реализует hal::rs* (см. lib/hal/include/hal/hal_rs485.h).
// Потокобезопасен: операции под мьютексом FreeRTOS. Режим SIMPLE_UART /
// MODBUS_RTU переключается на лету из любой задачи.

#include "driver_rs485.h"

#include <Arduino.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *RS_TAG = "RS485";

namespace drv {

using namespace rs485_config;

Rs485Driver &Rs485Driver::instance() {
    static Rs485Driver inst;
    return inst;
}

bool Rs485Driver::ensureLock() {
    if (!lock_) {
        lock_ = xSemaphoreCreateMutex();
    }
    return lock_ && xSemaphoreTake((SemaphoreHandle_t)lock_, pdMS_TO_TICKS(200)) == pdTRUE;
}

void Rs485Driver::releaseLock() {
    if (lock_) xSemaphoreGive((SemaphoreHandle_t)lock_);
}

void Rs485Driver::setTxMode(bool tx) {
    const int lvl = (tx == REDE_ACTIVE_HIGH) ? 1 : 0;
    gpio_set_level((gpio_num_t)PIN_REDE, lvl);
    // Небольшая задержка установления направления трансивера
    delayMicroseconds(5);
}

bool Rs485Driver::init() {
    if (inited_) return ready_;
    inited_ = true;

    // Пин RE/DE: выход, по умолчанию — приём (шина свободна)
    gpio_set_direction((gpio_num_t)PIN_REDE, GPIO_MODE_OUTPUT);
    setTxMode(false);

    uart_config_t cfg = {};
    cfg.baud_rate = (int)BAUD_RATE;
    cfg.data_bits = DATA_BITS;
    cfg.stop_bits = STOP_BITS;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.rx_flow_ctrl_thresh = 0;
#if defined(SOC_UART_HW_VER) && SOC_UART_HW_VER >= 2
    cfg.source_clk = UART_SCLK_DEFAULT; // IDF >= 5.x
#else
    cfg.source_clk = UART_SCLK_APB;     // IDF 4.x (Arduino core espressif32 6.x)
#endif

    esp_err_t err = uart_param_config(UART_PORT, &cfg);
    if (err == ESP_OK) err = uart_set_pin(UART_PORT, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err == ESP_OK) err = uart_driver_install(UART_PORT, RX_BUF_SIZE, TX_BUF_SIZE, 0, nullptr, 0);
    if (err != ESP_OK) {
        ESP_LOGE(RS_TAG, "UART init failed: %s", esp_err_to_name(err));
        ready_ = false;
        return false;
    }
    ready_ = true;
    ESP_LOGI(RS_TAG, "ready: uart=%d tx=%d rx=%d rede=%d baud=%u mode=%s",
             (int)UART_PORT, PIN_TX, PIN_RX, PIN_REDE, (unsigned)BAUD_RATE,
             mode_ == RsMode::MODBUS_RTU ? "MODBUS_RTU" : "SIMPLE_UART");
    return true;
}

void Rs485Driver::deinit() {
    if (!inited_) return;
    ensureLock();
    uart_flush(UART_PORT);
    uart_driver_delete(UART_PORT);
    setTxMode(false);
    ready_ = false;
    inited_ = false;
    releaseLock();
}

bool Rs485Driver::setMode(RsMode m) {
    if (!ensureLock()) return false;
    if (!ready_) { releaseLock(); return false; }
    if (mode_ == m) { releaseLock(); return true; }
    // Сбрасываем недочитанный приём — кадры старого протокола не нужны
    uart_flush_input(UART_PORT);
    mode_ = m;
    setTxMode(false); // направление — приём
    ESP_LOGI(RS_TAG, "mode switched -> %s (t3.5=%u ms)",
             m == RsMode::MODBUS_RTU ? "MODBUS_RTU" : "SIMPLE_UART",
             (unsigned)modbusT35Ms(BAUD_RATE));
    releaseLock();
    return true;
}

// Non-blocking peek: сколько байт лежит в RX-кольце driver/uart прямо сейчас.
static inline size_t rxPending(uart_port_t port) {
    size_t cnt = 0;
    // uart_get_buffered_data_len — стабильный API driver/uart (IDF 4.x и 5.x),
    // работает только после uart_driver_install (мы его устанавливаем в init()).
    if (uart_get_buffered_data_len(port, &cnt) != ESP_OK) return 0;
    return cnt;
}

int Rs485Driver::available() {
    if (!ready_) return 0;
    return (int)rxPending(UART_PORT);
}

size_t Rs485Driver::read(uint8_t *buf, size_t len, uint32_t timeoutMs) {
    if (!ready_ || !buf || len == 0) return 0;
    const int n = uart_read_bytes(UART_PORT, buf, len, pdMS_TO_TICKS(timeoutMs));
    return n > 0 ? (size_t)n : 0;
}

size_t Rs485Driver::write(const uint8_t *data, size_t len) {
    if (!ready_ || !data || len == 0) return 0;
    // Простой режим: потоковая запись без ручного управления RE/DE
    // (driver/uart сам удерживает TX, а трансивер после idle возвращается в RX)
    const int n = uart_write_bytes(UART_PORT, (const char *)data, len);
    return n > 0 ? (size_t)n : 0;
}

bool Rs485Driver::sendFrame(const uint8_t *data, size_t len) {
    if (!ensureLock()) return false;
    if (!ready_ || !data || len == 0) { releaseLock(); return false; }

    uint8_t tmp[256];
    const bool modbus = (mode_ == RsMode::MODBUS_RTU);
    if (modbus) {
        if (len + 2 > sizeof(tmp)) { releaseLock(); return false; }
        memcpy(tmp, data, len);
        const uint16_t crc = crc16(data, len);
        tmp[len] = (uint8_t)(crc & 0xFF);        // CRC low — first (Modbus)
        tmp[len + 1] = (uint8_t)(crc >> 8);
        len += 2;
    } else {
        if (len > sizeof(tmp)) { releaseLock(); return false; }
        memcpy(tmp, data, len);
    }

    // Модульный режим: вручную держим передатчик включённым до конца кадра
    if (modbus) setTxMode(true);
    const int n = uart_write_bytes(UART_PORT, (const char *)tmp, len);
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(1000));
    if (modbus) {
        delayMicroseconds(TX_GUARD_US);   // guard-time до перехода в приём
        setTxMode(false);
    }
    releaseLock();
    return n == (int)len;
}

size_t Rs485Driver::receiveFrame(uint8_t *buf, size_t maxLen, uint32_t timeoutMs) {
    if (!ensureLock()) return 0;
    if (!ready_ || !buf || maxLen == 0) { releaseLock(); return 0; }

    const bool modbus = (mode_ == RsMode::MODBUS_RTU);
    const uint32_t t35 = modbusT35Ms(BAUD_RATE);
    size_t got = 0;
    uint32_t deadline = millis() + timeoutMs;

    while ((int32_t)(deadline - millis()) > 0) {
        const size_t avail = (size_t)available();
        if (avail > 0) {
            const int n = uart_read_bytes(UART_PORT, buf + got,
                                          (int)(maxLen - got), pdMS_TO_TICKS(t35 + 2));
            if (n > 0) got += (size_t)n;
            if (got >= maxLen) break;
            if (!modbus) break; // простой режим: забрали всё, что пришло
        } else if (got > 0 && modbus) {
            // Кадр начинается: тишина дольше t3.5 => конец кадра
            vTaskDelay(pdMS_TO_TICKS(t35));
            if (available() == 0) break;
        } else if (got > 0) {
            break;
        } else {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

    if (modbus && got >= 4) {
        const uint16_t rxCrc = (uint16_t)buf[got - 2] | ((uint16_t)buf[got - 1] << 8);
        const uint16_t calc = crc16(buf, got - 2);
        if (rxCrc != calc) {
            ESP_LOGW(RS_TAG, "CRC mismatch: rx=0x%04X calc=0x%04X (%u bytes dropped)",
                     rxCrc, calc, (unsigned)got);
            got = 0; // битый кадр отбрасываем
        } else {
            got -= 2; // отсекаем CRC — выше возвращаем длину полезной части
        }
    }

    if (echo_ && got > 0) {
        Serial.printf("[RS485:%s] %u bytes:", modbus ? "MB" : "RAW", (unsigned)got);
        for (size_t i = 0; i < got && i < 32; ++i) Serial.printf(" %02X", buf[i]);
        Serial.println();
    }
    releaseLock();
    return got;
}

uint16_t Rs485Driver::crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

} // namespace drv

// ---- HAL-обёртки ----------------------------------------------------------------
namespace hal {

bool rsInit() { return drv::Rs485Driver::instance().init(); }
void rsDeinit() { drv::Rs485Driver::instance().deinit(); }
RsMode rsMode() { return drv::Rs485Driver::instance().mode(); }
bool rsSetMode(RsMode m) { return drv::Rs485Driver::instance().setMode(m); }
int rsAvailable() { return drv::Rs485Driver::instance().available(); }
size_t rsRead(uint8_t *buf, size_t len, uint32_t timeoutMs) {
    return drv::Rs485Driver::instance().read(buf, len, timeoutMs);
}
size_t rsWrite(const uint8_t *data, size_t len) {
    return drv::Rs485Driver::instance().write(data, len);
}
bool rsSendFrame(const uint8_t *data, size_t len) {
    return drv::Rs485Driver::instance().sendFrame(data, len);
}
size_t rsReceiveFrame(uint8_t *buf, size_t maxLen, uint32_t timeoutMs) {
    return drv::Rs485Driver::instance().receiveFrame(buf, maxLen, timeoutMs);
}
void rsEnableEcho(bool enable) { drv::Rs485Driver::instance().enableEcho(enable); }

} // namespace hal
