#include <unity.h>
#include <Arduino.h>
#include "project_config.h"
#include "gpio_config.h"

void test_gpio_config_valid(void) {
    // Компилируемая проверка таблицы пинов драйвера
    TEST_ASSERT_TRUE(gpio_config::validate());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(GpioPin::PIN_COUNT), gpio_config::kCount);
}

void test_status_led_is_output(void) {
    const auto &cfg = gpio_config::table[static_cast<int>(GpioPin::LED_STATUS)];
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(HalGpioMode::OUTPUT),
                            static_cast<uint8_t>(cfg.mode));
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
    RUN_TEST(test_status_led_is_output);
    RUN_TEST(test_fw_version_not_empty);
    UNITY_END();
}

void loop() {}
