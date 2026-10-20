# Wi-Fi в KOTELPC (ESP32)

Модуль: `lib/wifi` (`WifiManager`) + сервис настроек `lib/config_service` (NVS).

## Режимы работы

| Режим        | Когда включается                          | Параметры (src/config/project_config.h) |
|--------------|-------------------------------------------|------------------------------------------|
| **STA**      | В NVS сохранена непустая конфигурация Wi‑Fi | ssid/pass из `DeviceConfig.wifiSsid/wifiPassword` |
| **AP**       | SSID пуст или подключение к STA не удалось за 10 с | `AP_SSID = "KOTELPC-AP"`, `AP_PASSWORD = "12345678"` |

Логика выбора — в `WifiManager::begin()`: если SSID пуст → сразу AP; иначе попытка
подключения (ожидание до 10 000 мс), при неудаче — fallback в точку доступа.

## Хранение конфигурации

Настройки лежат в NVS, пространство имён `kotelpc`, ключ `config`, формат JSON
(ArduinoJson):

```json
{ "wifi_ssid": "MyNet", "wifi_pass": "secret", "poll_ms": 1000 }
```

API (`ConfigService`): `load(cfg)`, `save(cfg)`, `resetToDefaults(cfg)`.
Значения по умолчанию — `DEFAULT_WIFI_SSID` / `DEFAULT_WIFI_PASSWORD`
(пустые → при первой прошивке устройство стартует в режиме AP).

## Интерфейс WifiManager

```cpp
void begin(const String &ssid, const String &password); // STA + fallback AP
void startAccessPoint(const String &ssid, const String &password);
bool isConnected() const;   // true только в STA и WL_CONNECTED
String localIP() const;     // IP в STA-режиме
```

## Подключение пользователя

### Вариант 1 — через Serial-консоль (115200)

Пример обработчика команды (можно добавить в цикл чтения простой UART):

```cpp
// wifi <SSID> [PASSWORD]  -> сохранить в NVS и переподключиться
if (line.startsWith("wifi ")) {
    DeviceConfig cfg; ConfigService().load(cfg);
    auto parts = split(line.substring(5));           // ["SSID", "PASS"]
    cfg.wifiSsid  = parts[0];
    cfg.wifiPassword = parts.size() > 1 ? parts[1] : "";
    if (ConfigService().save(cfg)) {
        wifi.begin(String(cfg.wifiSsid.c_str()), String(cfg.wifiPassword.c_str()));
        Serial.println(wifi.isConnected() ? "OK " + wifi.localIP() : "FAIL -> AP mode");
    }
}
```

Сброс к значениям по умолчанию (пустой SSID → AP): `ConfigService().resetToDefaults(cfg); save(cfg);`

### Вариант 2 — через точку доступа

1. Подключиться к сети **KOTELPC-AP** (пароль `12345678`).
2. Открыть `http://192.168.4.1/` (стандартный адрес softAP ESP32).
3. Для веб-настройки необходимо поднять HTTP-сервер
   (`ESPAsyncWebServer` уже в `platformio.ini` lib_deps) и в его обработчике
   вызвать `ConfigService::save()` + `WifiManager::begin()` — сейчас сервер не запущен.

## Ограничения текущей реализации (v0.3.0)

- `WifiManager::begin()` **не вызывается** из `src/main.cpp` — Wi-Fi-стек при старте
  не поднимается; модуль готов к интеграции (нужно добавить инициализацию и задачу
  мониторинга соединения).
- Синхронное ожидание до 10 с внутри `begin()` блокирует вызывающую задачу; для
  FreeRTOS-кода предпочтительна отдельная задача Wi‑Fi с `vTaskDelay` вместо `delay()`.
- Нет автоматического переподключения при обрыве STA, нет mDNS/веб-интерфейса.

## Тестирование

- Проверка сборки после изменений: `pio run -e esp32dev`.
- Быстрая проверка на железе: зашить, открыть Serial 115200, убедиться, что без
  сохранённой конфигурации видна сеть `KOTELPC-AP`; затем отправить `wifi МояСеть пароль`
  (после добавления обработчика) и проверить `isConnected()/localIP()`.
