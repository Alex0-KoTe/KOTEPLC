#include <unity.h>
#include <Arduino.h>
#include "project_config.h"

void test_led_pin_defined(void) {
    TEST_ASSERT_EQUAL_UINT8(2, STATUS_LED_PIN);
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
    RUN_TEST(test_led_pin_defined);
    RUN_TEST(test_fw_version_not_empty);
    UNITY_END();
}

void loop() {}
