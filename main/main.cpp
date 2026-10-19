// KOTELPC — базовый скетч проекта на ESP32 (PlatformIO / Arduino framework)

#include <Arduino.h>
#include "project_config.h"

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    Serial.println();
    Serial.printf("KOTELPC boot. FW version: %s\n", FW_VERSION);
    Serial.printf("Chip: %s, rev %d, cores %d, freq %d MHz\n",
                  ESP.getChipModel(), ESP.getChipRevision(),
                  ESP.getChipCores(), ESP.getCpuFreqMHz());
}

void loop() {
    static uint32_t last_toggle = 0;

    // Сердцебиение: мигание светодиодом раз в секунду
    if (millis() - last_toggle >= SENSOR_READ_INTERVAL_MS) {
        last_toggle = millis();
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    }

    delay(LOOP_DELAY_MS);
}
