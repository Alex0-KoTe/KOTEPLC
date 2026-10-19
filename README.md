# KOTELPC

Прошивка для ESP32 (PlatformIO, Arduino framework): **FreeRTOS-задачи**, **LittleFS**
и доступ к железу через **драйверы с HAL**. Конфигурация GPIO находится в коде драйвера
(`lib/drivers/include/gpio_config.h`) — остальные модули знают только логические имена пинов.

## Архитектура

```
Прикладные задачи (main.cpp)          Heartbeat / Input / FsLog  — os::TaskBase (FreeRTOS)
        │
HAL (include/hal)                     hal::gpioRead/Write(логический пин)
        │
Драйверы (lib/drivers, lib/fs)        GpioDriver (ESP-IDF driver/gpio), FsDriver (joltwallet/esp_littlefs v1.14.8)
        │
Железо                                GPIO-пины, SPI Flash (раздел "storage")
```

- `GpioPin` — логические имена: LED_STATUS, RELAY_1, RELAY_2, BTN_1, SENSOR_1.
- Таблица `gpio_config::table[]` задаёт физический пин, режим, activeLow, начальное состояние;
  `static_assert(validate(), ...)` проверяет корректность на этапе компиляции
  (input-only пины 34–39 не используются как выходы, strapped-пины 6–11 — как нагрузки).
- LittleFS монтируется драйвером `lib/fs` с раздела `storage` из `partitions.csv`,
  операции потокобезопасны (mutex FreeRTOS).

## Структура проекта

```
├── platformio.ini              # env: esp32dev + partitions.csv + littlefs
├── partitions.csv              # таблица разделов (nvs/ota/storage/coredump)
├── include/
│   ├── project_config.h        # периоды задач, Wi-Fi, FW_VERSION
│   └── hal/hal_gpio.h          # интерфейс HAL GPIO
├── main/main.cpp               # setup(): инициализация драйверов + запуск задач
├── lib/
│   ├── drivers/                # ДРАЙВЕР GPIO
│   │   ├── include/gpio_config.h    # <-- КОНФИГУРАЦИЯ ПИНОВ ЗДЕСЬ
│   │   ├── include/driver_gpio.h
│   │   └── gpio_driver.cpp          # реализация hal::gpio* поверх ESP-IDF
│   ├── fs/                     # ДРАЙВЕР LittleFS (mount/read/write/log)
│   │   ├── include/fs_driver.h
│   │   └── fs_driver.cpp
│   └── wifi/wifi_manager.cpp
├── components/
│   ├── config_service/         # NVS + JSON конфигурация
│   └── freertos_task/include/freertos_task.h   # обёртка os::TaskBase
├── test/test_basic/            # Unity-тесты (валидация таблицы пинов)
└── data/                       # файлы для загрузки в LittleFS (pio run -t uploadfs)
```

## Сборка и прошивка

```bash
pio run                 # сборка
pio run -t upload       # прошивка (таблица разделов прошьётся автоматически)
pio run -t uploadfs     # загрузка содержимого data/ в LittleFS
pio device monitor      # монитор 115200
pio test -e esp32dev    # тесты
```

## Как поменять пин

1. Откройте `lib/drivers/include/gpio_config.h`.
2. Измените `gpio_num_t pin` у нужного сигнала в таблице (или добавьте новую строку
   + значение в enum `GpioPin`).
3. Соберите — `static_assert` не даст использовать недопустимый пин.

## Требования

- [PlatformIO Core](https://docs.platformio.org/page/installation.html)
- ESP32 dev board (esp32dev, 16 мБ flash)

