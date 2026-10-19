#include <unity.h>
#include <Arduino.h>
#include "project_config.h"
#include "gpio_config.h"
#include "mcp23017_config.h"

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
}

void test_mcp_i2c_pins(void) {
    TEST_ASSERT_EQUAL_UINT8(21, mcp_config::SDA_PIN);
    TEST_ASSERT_EQUAL_UINT8(22, mcp_config::SCL_PIN);
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
    RUN_TEST(test_mcp_i2c_pins);
    RUN_TEST(test_mcp_port_directions);
    RUN_TEST(test_mcp_pin_mapping);
    RUN_TEST(test_fw_version_not_empty);
    UNITY_END();
}

void loop() {}
