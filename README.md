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
│   ├── config_service/         # NVS + JSON конфигурация
│   └── freertos_task/include/freertos_task.h   # обёртка os::TaskBase
├── deps/esp_littlefs/          # драйвер LittleFS v1.14.8 (+ littlefs v2.5.1), скачан в проект
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

## Карта железа (KOTELPC)

| Сигнал | Пин/порт | Описание |
|---|---|---|
| SDA MCP23017 | IO21 | I2C data (100 кГц), конфиг в `lib/drivers/include/mcp23017_config.h` |
| SCL MCP23017 | IO22 | I2C clock, адрес чипа 0x20 |
| HEART LED | IO16 | пульсация «сердцебиения», конфиг в `lib/drivers/include/gpio_config.h` |
| UART TX | IO1 | отладочный Serial 115200 (UART0) |
| UART RX | IO3 | загрузка/прошивка (boot pin — использовать осторожно) |
| GPA0..GPA7 | MCP23017 A | релейные выходы RL0..RL7 (RELAY_ACTIVE_LOW настраивается в драйвере) |
| GPB0..GPB7 | MCP23017 B | дискретные входы DI0..DI7, внутренний pull-up, дебаунс 30 мс в драйвере |

Конфигурация GPIO живёт только в коде драйвера (`gpio_config.h`, `mcp23017_config.h`);
прикладной код обращается к железу через HAL (`hal::gpio*`, `hal::mcp*`) по логическим именам.

## Документация

- [docs/WIFI.md](docs/WIFI.md) — описание работы с Wi-Fi (режимы STA/AP, NVS-конфигурация, подключение).
