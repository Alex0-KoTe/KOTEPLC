// Драйвер расширителя GPIO MCP23017 (I2C, регистровая модель Bank0).
// Конфигурация шины, адреса и пинов — ИСКЛЮЧИТЕЛЬНО в mcp23017_config.h.
// Реализует hal::mcp* (см. lib/hal/include/hal/hal_mcp23017.h).
// Потокобезопасен: все обращения к шине под мьютексом FreeRTOS.

#include "driver_mcp23017.h"

#include <Arduino.h>
#include <Wire.h> // Wire (NG-драйвер Arduino) — единственный I2C мастер проекта
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <esp_log.h>

static const char *MCP_TAG = "MCP23017";

bool drv::Mcp23017Driver::sWireReady_ = false;

// ---- Регистры MCP23017 (Bank 0, CONFIG.BANK=0, дефолт после POR) ----
namespace reg {
constexpr uint8_t IODIRA  = 0x00;
constexpr uint8_t IODIRB  = 0x01;
constexpr uint8_t IPOLA   = 0x02;
constexpr uint8_t IPOLB   = 0x03;
constexpr uint8_t GPINTENA = 0x04;
constexpr uint8_t GPINTENB = 0x05;
constexpr uint8_t DEFVALA = 0x06;
constexpr uint8_t DEFVALB = 0x07;
constexpr uint8_t INTCONA = 0x08;
constexpr uint8_t INTCONB = 0x09;
constexpr uint8_t IOCONA  = 0x0A;
constexpr uint8_t IOCONB  = 0x0B;
constexpr uint8_t GPPUA   = 0x0C;
constexpr uint8_t GPPUB   = 0x0D;
constexpr uint8_t INTFA   = 0x0E;
constexpr uint8_t INTFB   = 0x0F;
constexpr uint8_t INTCAPA = 0x10;
constexpr uint8_t INTCAPB = 0x11;
constexpr uint8_t GPIOA   = 0x12;
constexpr uint8_t GPIOB   = 0x13;
constexpr uint8_t OLATA   = 0x14;
constexpr uint8_t OLATB   = 0x15;
} // namespace reg

namespace drv {

Mcp23017Driver &Mcp23017Driver::instance() {
    static Mcp23017Driver inst;
    return inst;
}

bool Mcp23017Driver::ensureLock() {
    if (!lock_) {
        lock_ = xSemaphoreCreateMutex();
    }
    return lock_ && xSemaphoreTake((SemaphoreHandle_t)lock_, pdMS_TO_TICKS(100)) == pdTRUE;
}

void Mcp23017Driver::releaseLock() {
    if (lock_) xSemaphoreGive((SemaphoreHandle_t)lock_);
}

// Единственный I2C мастер проекта — Wire (Arduino/NG-драйвер). ВАЖНО: в
// Arduino core для esp32dev Wire занимает ДВА аппаратных порта сразу
// (port 0 + port 1) — создание нативного мастера ESP-IDF на любом из них
// вызывает фатальный abort "CONFLICT! driver_ng is not allowed to be used
// with this old driver". Все транзакции идут только через Wire.
// Вызывается из init() ровно один раз; повторные вызовы безопасны.
void Mcp23017Driver::ensureWireBus() {
    if (sWireReady_) return;
    Wire.begin(mcp_config::SDA_PIN, mcp_config::SCL_PIN);
    Wire.setClock(mcp_config::CLOCK_HZ);
    Wire.setTimeOut(mcp_config::WIRE_TIMEOUT_MS);
    sWireReady_ = true;
}

// Одна запись регистра через Wire без вывода ошибок в лог.
bool Mcp23017Driver::writeRegQuiet(uint8_t addr, uint8_t value) {
    Wire.beginTransmission((uint8_t)mcp_config::I2C_ADDR);
    Wire.write(addr);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

// Чтение одного регистра через Wire без вывода ошибок в лог.
bool Mcp23017Driver::readRegQuiet(uint8_t addr, uint8_t &value) {
    Wire.beginTransmission((uint8_t)mcp_config::I2C_ADDR);
    Wire.write(addr);
    if (Wire.endTransmission(false) != 0) return false; // repeated start
    const int n = Wire.requestFrom((uint8_t)mcp_config::I2C_ADDR, (uint8_t)1);
    if (n != 1) { Wire.flush(); return false; }
    value = Wire.read();
    return true;
}

bool Mcp23017Driver::writeReg(uint8_t addr, uint8_t value) {
    ensureWireBus();
    const bool ok = writeRegQuiet(addr, value);
    if (!ok) ESP_LOGE(MCP_TAG, "write reg 0x%02X failed (Wire NACK/timeout)", addr);
    return ok;
}

bool Mcp23017Driver::readReg(uint8_t addr, uint8_t &value) {
    ensureWireBus();
    if (!readRegQuiet(addr, value)) {
        ESP_LOGE(MCP_TAG, "read reg 0x%02X failed (Wire NACK/timeout)", addr);
        return false;
    }
    return true;
}

// Результат проверки ACK на адресе: различаем «нет чипа» (NACK, 2),
// «шина цела, но транзакция сорвалась» (timeout/other) и успех (0).
// Реализация — через Wire (NG-драйвер): endTransmission() возвращает
// 2 = NACK on address, 1 = data NACK, 3/4 = timeout/other.
int Mcp23017Driver::probeBusDetailed(uint8_t addr) {
    ensureWireBus();
    Wire.beginTransmission(addr);
    const uint8_t err = Wire.endTransmission();
    if (err == 0) return 0;             // ACK
    if (err == 2) return 2;             // NACK by address
    return 4;                           // прочее (timeout и т.п.)
}

bool Mcp23017Driver::probeBus(uint8_t addr) {
    return probeBusDetailed(addr) == 0; // ACK от чипа => устройство на месте
}

// Явно гасим внутреннюю подтяжку ESP32 на пине (на плате есть внешние
// подтяжки SDA/SCL к +3V3, внутренняя не нужна и паразитна). Arduino-режим
// INPUT сам выключает pull-up, но делаем это гарантированно на уровне GPIO.
static void disablePullup(uint8_t pin) {
    gpio_pulldown_dis((gpio_num_t)pin);
    gpio_pullup_dis((gpio_num_t)pin);
}

// Проверка «залипания» линии SDA в LOW. ВАЖНО (FW 0.4.3): на плате есть
// ВНЕШНИЕ подтяжки SDA/SCL к +3V3, поэтому внутреннюю подтяжку ESP32
// (~45 кОм) НЕ включаем — режим INPUT_PULLUP паразитен при наличии внешней
// подтяжки. Линии проверяются в чисто высокоимпедансном режиме INPUT:
// свободная линия удерживается внешними резисторами в HIGH, LOW может быть
// только если линию держит чип или есть коротыш.
// Диагностика выполняется ДО Wire.begin() и НИКОГДА не трогает пины, если
// линии свободны. Причина: в Arduino core 2.0.17 для esp32dev активен
// NG-драйвер I2C (esp32-hal-i2c-ng.c) — он привязывает аппаратный порт к
// пинам на этапе Wire.begin(). Любое pinMode()/bit-bang по этим пинам ПОСЛЕ
// инициализации мастера сбрасывает состояние порта, и все последующие
// транзакции падают с «bus is not initialized».
bool Mcp23017Driver::sdaStuckLow() {
    // Только INPUT (high-Z): внешние подтяжки делают INPUT_PULLUP ненужным.
    pinMode(mcp_config::SDA_PIN, INPUT);
    pinMode(mcp_config::SCL_PIN, INPUT);
    disablePullup(mcp_config::SDA_PIN); // гарантированно гасим внутренний pull-up
    disablePullup(mcp_config::SCL_PIN);
    delayMicroseconds(50); // дать линиям установиться через внешние подтяжки
    const bool sda_low = digitalRead(mcp_config::SDA_PIN) == LOW;
    const bool scl_low = digitalRead(mcp_config::SCL_PIN) == LOW;
    if (!sda_low && !scl_low) {
        // Линии свободны. Возвращаем пины в INPUT (высокоимпеданное состояние
        // с внешней подтяжкой): Wire.begin() сразу после этого назначит их на
        // I2C-функцию корректно, без всяких пересозданий мастера.
        pinMode(mcp_config::SDA_PIN, INPUT);
        pinMode(mcp_config::SCL_PIN, INPUT);
        return false;
    }
    if (scl_low) {
        ESP_LOGE(MCP_TAG, "SCL (IO%d) stuck LOW at boot — check wiring/short", mcp_config::SCL_PIN);
    }
    return sda_low;
}

// Стандартная процедура восстановления I2C: до 9 тактов SCL + STOP,
// чтобы «отпустить» slave, застрявший в середине транзакции.
// Вызывается ТОЛЬКО когда линии реально залипли (пины ещё не отданы
// I2C-мастеру) — после bit-bang мастер создаётся заново ровно один раз.
void Mcp23017Driver::recoverBus() {
    ESP_LOGW(MCP_TAG, "SDA stuck LOW, clock recovery (9x SCL + STOP)");
    pinMode(mcp_config::SCL_PIN, OUTPUT);
    pinMode(mcp_config::SDA_PIN, INPUT); // внешние подтяжки: внутреннюю не включаем
    for (int i = 0; i < 9; ++i) {
        digitalWrite(mcp_config::SCL_PIN, HIGH);
        delayMicroseconds(5);
        digitalWrite(mcp_config::SCL_PIN, LOW);
        delayMicroseconds(5);
        if (digitalRead(mcp_config::SDA_PIN) == HIGH) break; // slave отпустил линию
    }
    // STOP: SDA -> HIGH при HIGH SCL
    digitalWrite(mcp_config::SCL_PIN, HIGH);
    delayMicroseconds(5);
    pinMode(mcp_config::SDA_PIN, OUTPUT);
    digitalWrite(mcp_config::SDA_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(mcp_config::SDA_PIN, HIGH);
    delayMicroseconds(5);
    // Пинов касаемся только здесь — после этого Wire.begin() переназначит их
    // на I2C-функцию (см. init(): единственный повторный begin в проекте).
}

bool Mcp23017Driver::init() {
    // Повторный вызов (например, после fallback) не должен пересоздавать
    // мастер и гонять диагностику шины — чип уже проверен.
    if (ready_) return true;

    // Диагностика линий ДО создания мастера (см. комментарий к sdaStuckLow):
    // после инициализации I2C-порта трогать пины нельзя.
    const bool sda_low = sdaStuckLow();
    if (sda_low) {
        recoverBus();
    }

    // Единственный мастер — Wire (NG-драйвер). Нативный ESP-IDF мастер
    // использовать нельзя: Wire занимает порты 0 И 1, и попытка создать
    // свой мастер приводит к фатальному "CONFLICT! driver_ng is not
    // allowed to be used with this old driver" (FW 0.4.4 -> 0.4.5).
    if (!ensureLock()) return false;
    ensureWireBus();
    releaseLock();
    if (!sda_low) {
        ESP_LOGI(MCP_TAG, "bus lines OK (SDA/SCL idle HIGH), Wire master @sda=%d scl=%d %u Hz",
                 mcp_config::SDA_PIN, mcp_config::SCL_PIN, (unsigned)mcp_config::CLOCK_HZ);
    }

    // Проверка присутствия чипа: адрес должен ответить ACK'ом.
    // Первая транзакция после включения питания может сорваться (POR чипа,
    // заряд ёмкостей) — поэтому до 3 быстрых попыток с паузой 5 мс.
    if (!ensureLock()) return false;
    int ackCode = 2; // 2 = NACK (никто не ответил) по умолчанию
    for (int i = 0; i < 3; ++i) {
        ackCode = probeBusDetailed(mcp_config::I2C_ADDR);
        if (ackCode == 0) break;
        delay(5);
    }
    bool present = false;
    if (ackCode == 0) {
        // Проверка регистровой модели: запись IOCON должна считаться обратно.
        // До 5 попыток с паузой 20 мс: после фиктивного START+STOP на старте
        // Wire чип может «просыпаться» не с первой транзакции. Quiet-версии
        // не засоряют лог ошибками при переборах.
        for (int i = 0; i < 5 && !present; ++i) {
            uint8_t v = 0xFF;
            present = writeRegQuiet(reg::IOCONA, 0x00) && readRegQuiet(reg::IOCONA, v) && v == 0x00;
            if (!present) delay(20);
        }
    }
    releaseLock();

    if (!present) {
        // Краткая честная диагностика: ACK-опрос только адресов MCP23017
        // (0x20..0x27). Полный скан при зависшей шине «съедает» секунды.
        char found[48] = "";
        size_t pos = 0;
        if (!ensureLock()) return false;
        for (uint8_t a = 0x20; a <= 0x27; ++a) {
            if (probeBusDetailed(a) == 0 && pos < sizeof(found) - 8) {
                pos += snprintf(found + pos, sizeof(found) - pos, "0x%02X ", a);
            }
        }
        releaseLock();
        ESP_LOGE(MCP_TAG, "init failed at 0x%02X (ack=%d)%s%s devices with ACK: %s",
                 mcp_config::I2C_ADDR, ackCode,
                 sda_low ? " [SDA was stuck LOW at boot]" : "",
                 ackCode == 0 ? " [ACK but IOCON verify failed]" : "",
                 found[0] ? found : "none");
        return false;
    }

    // Инициализируем latch реле выключенными ДО переключения направления
    // портов (чтобы при старте не было ложных срабатываний RL0..RL7).
    if (!ensureLock()) return false;
    bool ok = writeReg(reg::OLATA, 0x00);
    releaseLock();
    if (!ok) return false;

    // Дебаунс: сырое/стабилизированное состояние входов = "все разомкнуты"
    // (при ACTIVE_LOW_INPUT уровень HIGH = неактивно)
    rawB_ = 0xFF;
    stableB_ = 0xFF;

    // Конфигурация портов из таблицы драйвера
    if (!ensureLock()) return false;
    ok = writeReg(reg::IODIRA, mcp_config::IODIRA) &&
         writeReg(reg::IODIRB, mcp_config::IODIRB) &&
         writeReg(reg::GPPUA,  mcp_config::GPPUA)  &&
         writeReg(reg::GPPUB,  mcp_config::GPPUB)  &&
              writeReg(reg::IPOLA,  0x00) &&
              writeReg(reg::IPOLB,  0x00) &&
              writeReg(reg::GPINTENA, 0x00) && // прерывания по изменению выкл.
              writeReg(reg::GPINTENB, 0x00) &&
              writeReg(reg::OLATA,  0x00) &&
              writeReg(reg::OLATB,  0x00);
    releaseLock();

    if (!ok) {
        ESP_LOGE(MCP_TAG, "port configuration failed");
        return false;
    }

    ready_ = true;
    ESP_LOGI(MCP_TAG, "ready @0x%02X (sda=%u scl=%u, %u Hz)",
             mcp_config::I2C_ADDR, mcp_config::SDA_PIN, mcp_config::SCL_PIN,
             (unsigned)mcp_config::CLOCK_HZ);
    return true;
}

bool Mcp23017Driver::isValid(McpPin pin) {
    const auto v = static_cast<uint8_t>(pin);
    return v <= 0x07 || (v >= 0x10 && v <= 0x17);
}

uint8_t Mcp23017Driver::portOf(McpPin pin) { return (static_cast<uint8_t>(pin) >> 4) & 1; }
uint8_t Mcp23017Driver::bitOf(McpPin pin)  { return static_cast<uint8_t>(pin) & 0x07; }

bool Mcp23017Driver::write(McpPin pin, bool high) {
    if (!ready_ || !isValid(pin)) return false;
    const uint8_t port = portOf(pin);
    const uint8_t bit = bitOf(pin);
    const uint8_t olat = port ? reg::OLATB : reg::OLATA;

    if (!ensureLock()) return false;
    uint8_t cur = 0;
    bool ok = readReg(olat, cur);
    if (ok) {
        cur = high ? (cur | (1 << bit)) : (cur & ~(1 << bit));
        ok = writeReg(olat, cur);
    }
    releaseLock();
    return ok;
}

bool Mcp23017Driver::read(McpPin pin) const {
    if (!ready_ || !isValid(pin)) return false;
    const uint8_t port = portOf(pin);
    const uint8_t bit = bitOf(pin);
    const uint8_t gpio = port ? reg::GPIOB : reg::GPIOA;

    uint8_t val = 0;
    if (!const_cast<Mcp23017Driver *>(this)->ensureLock()) return false;
    const bool ok = const_cast<Mcp23017Driver *>(this)->readReg(gpio, val);
    const_cast<Mcp23017Driver *>(this)->releaseLock();
    if (!ok) return false;
    return (val >> bit) & 1;
}

bool Mcp23017Driver::writePort(uint8_t port, uint8_t value) {
    if (!ready_) return false;
    if (!ensureLock()) return false;
    const bool ok = writeReg(port ? reg::OLATB : reg::OLATA, value);
    releaseLock();
    return ok;
}

bool Mcp23017Driver::readPort(uint8_t port, uint8_t &value) {
    if (!ready_) return false;
    if (!ensureLock()) return false;
    const bool ok = readReg(port ? reg::GPIOB : reg::GPIOA, value);
    releaseLock();
    return ok;
}


bool Mcp23017Driver::relaySet(McpPin pin, bool active) {
    if (!isValid(pin) || portOf(pin) != 0) return false; // реле — только порт A
    const bool level = mcp_config::RELAY_ACTIVE_LOW ? !active : active;
    return write(pin, level);
}

// ---- Дебаунс входов DI0..DI7 (порт B) ----
// Машина дебаунса: сырое слово порта B читается пакетно одним обращением;
// стабилизированное значение stableB_ обновляется только когда сырое значение
// не меняется дольше DEBOUNCE_MS. Храним "уровни" (бит = уровень пина),
// инверсию по ACTIVE_LOW_INPUT применяем на выходе.
void Mcp23017Driver::pollInputs() {
    if (!ready_) return;
    uint8_t val = 0xFF;
    if (!readPort(1, val)) return;         // ошибка шины — состояние не меняем
    if (val != rawB_) {                    // перешёл дребезг/изменение контакта
        rawB_ = val;
        for (uint8_t i = 0; i < 8; ++i) changeTick_[i] = millis();
    } else {
        const uint32_t now = millis();
        if (val != stableB_ && (now - changeTick_[0]) >= mcp_config::DEBOUNCE_MS) {
            stableB_ = val;                // уровень держится стабильно — фиксируем
        }
    }
}

uint8_t Mcp23017Driver::debouncedState() const {
    // Слово активных входов: бит i = 1 => DIi активен (контакт замкнут на GND).
    // До первой успешной инициализации/опроса считаем все входы неактивными.
    if (!ready_ || stableB_ == 0xFF) return 0x00;
    return mcp_config::ACTIVE_LOW_INPUT ? static_cast<uint8_t>(~stableB_) : stableB_;
}

bool Mcp23017Driver::diActive(McpPin pin) const {
    if (!ready_ || !isValid(pin) || portOf(pin) != 1) return false; // DI — только порт B
    return (debouncedState() >> bitOf(pin)) & 1;
}

} // namespace drv

// ---- Реализация HAL ----
namespace hal {

bool mcpInit() { return drv::Mcp23017Driver::instance().init(); }
bool mcpIsReady() { return drv::Mcp23017Driver::instance().isReady(); }

bool mcpWrite(int logicalPin, bool high) {
    return drv::Mcp23017Driver::instance().write(static_cast<McpPin>(logicalPin), high);
}

bool mcpRead(int logicalPin) {
    return drv::Mcp23017Driver::instance().read(static_cast<McpPin>(logicalPin));
}

bool mcpRelaySet(int logicalPin, bool active) {
    return drv::Mcp23017Driver::instance().relaySet(static_cast<McpPin>(logicalPin), active);
}

bool mcpDiActive(int logicalPin) {
    return drv::Mcp23017Driver::instance().diActive(static_cast<McpPin>(logicalPin));
}

bool mcpWritePortA(uint8_t value) { return drv::Mcp23017Driver::instance().writePort(0, value); }
bool mcpWritePortB(uint8_t value) { return drv::Mcp23017Driver::instance().writePort(1, value); }
bool mcpReadPortA(uint8_t &value) { return drv::Mcp23017Driver::instance().readPort(0, value); }
bool mcpReadPortB(uint8_t &value) { return drv::Mcp23017Driver::instance().readPort(1, value); }

void mcpPollInputs() { drv::Mcp23017Driver::instance().pollInputs(); }
uint8_t mcpDebouncedState() { return drv::Mcp23017Driver::instance().debouncedState(); }

} // namespace hal
