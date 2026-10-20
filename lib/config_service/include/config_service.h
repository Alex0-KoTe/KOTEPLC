#ifndef CONFIG_SERVICE_H
#define CONFIG_SERVICE_H

#include <WiFi.h>
#include <ArduinoJson.h>
#include <string>

// Сервис конфигурации: хранение настроек в NVS (Preferences).
struct DeviceConfig {
    std::string wifiSsid;
    std::string wifiPassword;
    int pollIntervalMs = 1000;
};

class ConfigService {
public:
    bool load(DeviceConfig &cfg);
    bool save(const DeviceConfig &cfg);
    bool resetToDefaults(DeviceConfig &cfg);

private:
    static constexpr const char *NVS_NAMESPACE = "kotelpc";
    static constexpr const char *NVS_KEY = "config";
};

#endif // CONFIG_SERVICE_H
