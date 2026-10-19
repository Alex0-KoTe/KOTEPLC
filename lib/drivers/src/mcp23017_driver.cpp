// Драйвер расширителя GPIO MCP23017 (I2C, регистровая модель Bank0).
// Конфигурация шины, адреса и пинов — ИСКЛЮЧИТЕЛЬНО в mcp23017_config.h.
// Реализует hal::mcp* (см. lib/hal/include/hal/hal_mcp23017.h).
// Потокобезопасен: все обращения к шине под мьютексом FreeRTOS.

#include "driver_mcp23017.h"

#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <esp_log.h>

static const char *MCP_TAG = "MCP23017";

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

bool Mcp23017Driver::writeReg(uint8_t addr, uint8_t value) {
    Wire.beginTransmission(mcp_config::I2C_ADDR);
    Wire.write(addr);
    Wire.write(value);
    const bool ok = Wire.endTransmission() == 0;
    if (!ok) {
        ESP_LOGE(MCP_TAG, "write reg 0x%02X failed", addr);
    }
    return ok;
}

bool Mcp23017Driver::readReg(uint8_t addr, uint8_t &value) {
    if (Wire.requestFrom((int)mcp_config::I2C_ADDR, (int)addr, 1) != 1) {
        ESP_LOGE(MCP_TAG, "read reg 0x%02X failed", addr);
        return false;
    }
    value = Wire.read();
    return true;
}

// Результат проверки ACK на адресе: различаем «нет чипа» (NACK, 2),
// «шина цела, но транзакция сорвалась» (timeout/other) и успех (0).
int Mcp23017Driver::probeBusDetailed(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission();
}

bool Mcp23017Driver::probeBus(uint8_t addr) {
    return probeBusDetailed(addr) == 0; // ACK от чипа => устройство на месте
}

// Проверка «залипания» линии SDA в LOW. Линию временно переводим в режим
// INPUT_PULLUP (внутренняя подтяжка ~45 кОм): если даже с ней уровень LOW —
// линию действительно держит чип или коротыш.
// ВАЖНО (FW 0.4.2): эта диагностика выполняется ДО Wire.begin() и НИКОГДА
// не трогает пины, если линии свободны. Причина: в Arduino core 2.0.17 для
// esp32dev активен NG-драйвер I2C (esp32-hal-i2c-ng.c) — он привязывает
// аппаратный порт к пинам на этапе Wire.begin(). Любое pinMode()/bit-bang
// по этим пинам ПОСЛЕ инициализации мастера сбрасывает состояние порта, и
// все последующие транзакции падают с «bus is not initialized» (ровно это
// наблюдали в логах: ACK на скане был, а записи IOCON — нет).
bool Mcp23017Driver::sdaStuckLow() {
    pinMode(mcp_config::SDA_PIN, INPUT_PULLUP);
    pinMode(mcp_config::SCL_PIN, INPUT_PULLUP);
    delayMicroseconds(50); // дать линиям установиться через подтяжки
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
    pinMode(mcp_config::SDA_PIN, INPUT_PULLUP);
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
    // после Wire.begin() трогать пины нельзя — NG-драйвер теряет порт.
    const bool sda_low = sdaStuckLow();
    if (sda_low) {
        recoverBus();
    }

    Wire.begin(mcp_config::SDA_PIN, mcp_config::SCL_PIN, mcp_config::CLOCK_HZ);
    Wire.setTimeOut(mcp_config::WIRE_TIMEOUT_MS);
    if (sda_low) {
        // После bit-bang мастер пересоздаём ровно один раз (end+begin
        // идемпотентны именно в этой точке: порт ещё не переиспользовался).
        Wire.end();
        Wire.begin(mcp_config::SDA_PIN, mcp_config::SCL_PIN, mcp_config::CLOCK_HZ);
        Wire.setTimeOut(mcp_config::WIRE_TIMEOUT_MS);
    } else {
        ESP_LOGI(MCP_TAG, "bus lines OK (SDA/SCL idle HIGH), master @sda=%d scl=%d %u Hz",
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
        // Повторяем до 3 раз: первая чтение/запись после POR иногда теряется.
        for (int i = 0; i < 3 && !present; ++i) {
            uint8_t v = 0xFF;
            present = writeReg(reg::IOCONA, 0x00) && readReg(reg::IOCONA, v) && v == 0x00;
            if (!present) delay(5);
        }
    }
    releaseLock();

    if (!present) {
        // Честная диагностика: скан только адресов MCP23017 (0x20..0x27) —
        // полный скан 0x08..0x77 при зависшей шине «съедает» ~50 мс на каждый
        // timeout и блокирует старт на десятки секунд.
        char found[48] = "";
        size_t pos = 0;
        int lastCode = ackCode;
        bool scanAcked = false;
        if (!ensureLock()) return false;
        for (uint8_t a = 0x20; a <= 0x27; ++a) {
            lastCode = probeBusDetailed(a);
            if (lastCode == 0) {
                scanAcked = true;
                if (pos < sizeof(found) - 8) {
                    pos += snprintf(found + pos, sizeof(found) - pos, "0x%02X ", a);
                }
            }
        }
        releaseLock();

        if (scanAcked && !sda_low) {
            // Шина жива и кто-то ACK'ует (скорее всего это наш чип на 0x20),
            // но IOCON не считался обратно. Типичная причина: регистр IOCON
            // записан в BANK/SLEW-режиме (Bank1) предыдущей прошивкой или
            // бит SEQOP/IOCON_LOCK мешает verify. Пробуем вариант Bank1
            // (IOCONB=0x15), а затем мягкий сброс через повторную запись.
            uint8_t v1 = 0xFF;
            bool bank1ok = false;
            if (!ensureLock()) return false;
            bank1ok = writeReg(0x15, 0x00) && readReg(0x15, v1) && v1 == 0x00;
            if (!bank1ok) {
                // Последняя попытка: IOCONA c битом BANK=1 -> читаем IOCONB
                writeReg(reg::IOCONA, 0x00);
                delay(2);
                bank1ok = readReg(reg::IOCONA, v1); // любой ответ = регистр жив
            }
            releaseLock();
            if (bank1ok) {
                ESP_LOGW(MCP_TAG, "chip at 0x%02X ACKs but IOCON verify failed; "
                         "accepted as present (devices with ACK: %s)",
                         mcp_config::I2C_ADDR, found);
                // Не считаем чип готовым автоматически — main сделает retry;
                // но diagnostic-строка теперь честная.
            }
            ESP_LOGE(MCP_TAG, "no verified chip at 0x%02X (ack=%d) | devices with ACK: %s",
                     mcp_config::I2C_ADDR, ackCode, found);
        } else if (found[0]) {
            ESP_LOGE(MCP_TAG, "no verified chip at 0x%02X (ack=%d) | devices with ACK: %s%s",
                     mcp_config::I2C_ADDR, ackCode, found,
                     sda_low ? " | SDA was stuck LOW at boot" : "");
        } else if (ackCode == 0) {
            // Адрес ACK'ует, но регистровая модель не подтверждена:
            // вероятнее всего чип в середине POR или линии деградированы.
            ESP_LOGE(MCP_TAG, "0x%02X ACKs but IOCON verify failed%s",
                     mcp_config::I2C_ADDR, sda_low ? " | SDA was stuck LOW at boot" : "");
        } else {
            ESP_LOGE(MCP_TAG, "chip not found at 0x%02X (ack code=%d: 0=OK 1=dataNAK 2=NACK 3=arb 4=timeout), scan 0x20..0x27 last=%d%s",
                     mcp_config::I2C_ADDR, ackCode, lastCode,
                     sda_low ? " | SDA was stuck LOW at boot" : "");
        }
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

bool Mcp23017Driver::diActive(McpPin pin) const {
    if (!ready_ || !isValid(pin) || portOf(pin) != 1) return false; // DI — только порт B
    const uint8_t bit = bitOf(pin);
    const bool rawLevel = (stableB_ >> bit) & 1;
    // вход активен при LOW (сухой контакт на GND), если ACTIVE_LOW_INPUT
    return mcp_config::ACTIVE_LOW_INPUT ? !rawLevel : rawLevel;
}

void Mcp23017Driver::pollInputs() {
    if (!ready_) return;
    uint8_t raw = 0;
    if (!readPort(1, raw)) return; // порт B

    const uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint8_t changedMask = 0;

    if (raw != rawB_) {
        // сбрасываем таймаут стабильности только для изменившихся битов
        const uint8_t diff = raw ^ rawB_;
        for (uint8_t b = 0; b < 8; ++b) {
            if ((diff >> b) & 1) changeTick_[b] = now;
        }
        rawB_ = raw;
    }

    for (uint8_t b = 0; b < 8; ++b) {
        const bool rawBit = (rawB_ >> b) & 1;
        const bool stableBit = (stableB_ >> b) & 1;
        if (rawBit == stableBit) continue;
        if (now - changeTick_[b] >= mcp_config::DEBOUNCE_MS) {
            stableB_ = (uint8_t)((stableB_ & ~(1 << b)) | (rawBit ? (1 << b) : 0));
            changedMask |= (1 << b);
        }
    }

    if (changedMask) {
        ESP_LOGI(MCP_TAG, "DI state changed: mask=0x%02X stable=0x%02X",
                 changedMask, stableB_);
    }
}

} // namespace drv

// ---- Реализация HAL ----
namespace hal {

bool mcpInit() { return drv::Mcp23017Driver::instance().init(); }
bool mcpIsReady() { return drv::Mcp23017Driver::instance().isReady(); }

// Fallback-процедура: если предыдущая попытка оборвалась на середине
// транзакции (или чип залип в Bank1/SlewRate от неудачной записи IOCON),
// адрес может ACK'овать, но регистры читаться неверно. Пишем оба IOCON по
// обоим банковским адресам (0x0A/0x0B и 0x14/0x15) со значением 0x00 —
// это возвращает микросхему в Bank0 с включёнными push-pull выходами.
void mcpForceInitFallback() {
    auto &mcp = drv::Mcp23017Driver::instance();
    Wire.beginTransmission(mcp_config::I2C_ADDR);
    Wire.write(0x0A); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(mcp_config::I2C_ADDR);
    Wire.write(0x14); Wire.write(0x00); Wire.endTransmission(); // IOCON при BANK=1
    delay(2);
    mcp.init(); // повторная полная проверка
}

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

void mcpPollInputs() {
    drv::Mcp23017Driver::instance().pollInputs();
}

// [DIAG-PULLUP] сырые регистровые операции для самодиагностики (см. hal_mcp23017.h)
bool mcpDiagWriteReg(uint8_t addr, uint8_t value) {
    auto &mcp = drv::Mcp23017Driver::instance();
    if (!mcp.diagEnsureLock()) return false;
    const bool ok = mcp.diagWriteReg(addr, value);
    mcp.diagReleaseLock();
    return ok;
}
bool mcpDiagReadReg(uint8_t addr, uint8_t &value) {
    auto &mcp = drv::Mcp23017Driver::instance();
    if (!mcp.diagEnsureLock()) return false;
    const bool ok = mcp.diagReadReg(addr, value);
    mcp.diagReleaseLock();
    return ok;
}

bool mcpWritePortA(uint8_t value) { return drv::Mcp23017Driver::instance().writePort(0, value); }
bool mcpWritePortB(uint8_t value) { return drv::Mcp23017Driver::instance().writePort(1, value); }
bool mcpReadPortA(uint8_t &value) { return drv::Mcp23017Driver::instance().readPort(0, value); }
bool mcpReadPortB(uint8_t &value) { return drv::Mcp23017Driver::instance().readPort(1, value); }

} // namespace hal
