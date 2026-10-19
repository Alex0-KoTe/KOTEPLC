#include <unity.h>
#include <Arduino.h>
#include "project_config.h"
#include "gpio_config.h"
#include "mcp23017_config.h"
#include "rs485_config.h"
#include "hal/hal_rs485.h"
#include "driver_rs485.h"

void test_gpio_config_valid(void) {
    // Компилируемая проверка таблицы пинов драйвера (включая резервирование
    // IO21/IO22 под I2C и IO1/IO3 под UART)
    TEST_ASSERT_TRUE(kGpioConfig::validate());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(GpioPin::PIN_COUNT), kGpioConfig::kCount);
}

void test_heart_led_is_output_io16(void) {
    const auto &cfg = kGpioConfig::table[static_cast<int>(GpioPin::LED_HEART)];
    TEST_ASSERT_EQUAL_INT(GPIO_NUM_16, static_cast<int>(cfg.pin));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(HalGpioMode::OUTPUT_MODE),
                            static_cast<uint8_t>(cfg.mode));
    // LED включается подачей логического 0 на IO16 => activeLow
    TEST_ASSERT_TRUE(cfg.activeLow);
}

void test_periph_en_is_output_io4(void) {
    const auto &cfg = kGpioConfig::table[static_cast<int>(GpioPin::PERIPH_EN)];
    TEST_ASSERT_EQUAL_INT(GPIO_NUM_4, static_cast<int>(cfg.pin));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(HalGpioMode::OUTPUT_MODE),
                            static_cast<uint8_t>(cfg.mode));
    // Логическая 1 на IO4 включает питание периферии (activeHigh)
    TEST_ASSERT_FALSE(cfg.activeLow);
    // На старте (gpioInit) периферия обесточена — включение после старта
    TEST_ASSERT_FALSE(cfg.initialHigh);
}

void test_mcp_i2c_pins(void) {
    TEST_ASSERT_EQUAL_UINT8(21, mcp_config::SDA_PIN);
    TEST_ASSERT_EQUAL_UINT8(22, mcp_config::SCL_PIN);
    // Все пины ADDR (A2 A1 A0) притянуты к минусу -> адрес 0x20
    TEST_ASSERT_EQUAL_UINT8(0x20, mcp_config::I2C_ADDR);
}

void test_mcp_port_directions(void) {
    // Порт A — выходы (реле RL0..RL7): бит направления = 0
    TEST_ASSERT_EQUAL_UINT8(0x00, mcp_config::IODIRA);
    // Порт B — входы (DI0..DI7) с подтяжками
    TEST_ASSERT_EQUAL_UINT8(0xFF, mcp_config::IODIRB);
    TEST_ASSERT_EQUAL_UINT8(0xFF, mcp_config::GPPUB);
}

void test_mcp_pin_mapping(void) {
    // GPA0..GPA7 -> RL0..RL7
    TEST_ASSERT_EQUAL_UINT8(0x00, static_cast<uint8_t>(McpPin::RL0));
    TEST_ASSERT_EQUAL_UINT8(0x07, static_cast<uint8_t>(McpPin::RL7));
    // GPB0..GPB7 -> DI0..DI7
    TEST_ASSERT_EQUAL_UINT8(0x10, static_cast<uint8_t>(McpPin::DI0));
    TEST_ASSERT_EQUAL_UINT8(0x17, static_cast<uint8_t>(McpPin::DI7));
}

void test_rs485_pins(void) {
    // IO1=TX, IO3=RX (UART), IO23=RE/DE направление передачи
    TEST_ASSERT_EQUAL_INT(1, rs485_config::PIN_TX);
    TEST_ASSERT_EQUAL_INT(3, rs485_config::PIN_RX);
    TEST_ASSERT_EQUAL_INT(23, rs485_config::PIN_REDE);
    // RE/DE зарезервирован в таблице GPIO-драйвера (не может быть назначен как GPIO)
    bool found = false;
    for (size_t i = 0; i < sizeof(kGpioConfig::kReservedPins) / sizeof(int); ++i) {
        if (kGpioConfig::kReservedPins[i] == rs485_config::PIN_REDE) found = true;
    }
    TEST_ASSERT_TRUE(found);
    TEST_ASSERT_TRUE(kGpioConfig::validate());
}

void test_modbus_crc16(void) {
    // Эталонный кадр Modbus: запрос холдинг-регистра 0 слейва 1
    const uint8_t req[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
    TEST_ASSERT_EQUAL_UINT16(0x0AC4, drv::Rs485Driver::crc16(req, sizeof(req)));
    // Кадр ответа слейва 1: 01 03 02 00 0A + CRC
    const uint8_t resp[] = {0x01, 0x03, 0x02, 0x00, 0x0A, 0xC4, 0x0B};
    TEST_ASSERT_EQUAL_UINT16(
        (uint16_t)resp[5] | ((uint16_t)resp[6] << 8),
        drv::Rs485Driver::crc16(resp, 5));
}

void test_modbus_t35(void) {
    // t3.5 при 19200 бод ~ 2 мс (минимум по спецификации), при 9600 ~ 5 мс
    TEST_ASSERT_EQUAL_UINT32(2, rs485_config::modbusT35Ms(19200));
    TEST_ASSERT_TRUE(rs485_config::modbusT35Ms(9600) >= 4);
    TEST_ASSERT_TRUE(rs485_config::modbusT35Ms(115200) >= 2);
}

void test_rs_mode_switch_values(void) {
    // Значения режима стабильны (используются в NVS/протоколе)
    TEST_ASSERT_EQUAL_UINT8(0, static_cast<uint8_t>(RsMode::SIMPLE_UART));
    TEST_ASSERT_EQUAL_UINT8(1, static_cast<uint8_t>(RsMode::MODBUS_RTU));
}

void test_fw_version_not_empty(void) {
    TEST_ASSERT_NOT_NULL(FW_VERSION);
    TEST_ASSERT_GREATER_THAN_UINT32(0, strlen(FW_VERSION));
}

void setUp(void) {}
void tearDown(void) {}

void setup() {
    delay(2000); // ожидание порта для отладки

    UNITY_BEGIN();
    RUN_TEST(test_gpio_config_valid);
    RUN_TEST(test_heart_led_is_output_io16);
    RUN_TEST(test_periph_en_is_output_io4);
    RUN_TEST(test_mcp_i2c_pins);
    RUN_TEST(test_mcp_port_directions);
    RUN_TEST(test_mcp_pin_mapping);
    RUN_TEST(test_rs485_pins);
    RUN_TEST(test_modbus_crc16);
    RUN_TEST(test_modbus_t35);
    RUN_TEST(test_rs_mode_switch_values);
    RUN_TEST(test_fw_version_not_empty);
    UNITY_END();
}

void loop() {}
