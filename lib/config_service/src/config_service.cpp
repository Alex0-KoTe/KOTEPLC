#include "config_service.h"
#include <Preferences.h>
#include <Arduino.h>

bool ConfigService::load(DeviceConfig &cfg) {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) {
        return false;
    }
    String json = prefs.getString(NVS_KEY, "");
    prefs.end();

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);
    if (err) {
        return false;
    }

    cfg.wifiSsid = doc["wifi_ssid"] | "";
    cfg.wifiPassword = doc["wifi_pass"] | "";
    cfg.pollIntervalMs = doc["poll_ms"] | 1000;
    return true;
}

bool ConfigService::save(const DeviceConfig &cfg) {
    JsonDocument doc;
    doc["wifi_ssid"] = cfg.wifiSsid;
    doc["wifi_pass"] = cfg.wifiPassword;
    doc["poll_ms"] = cfg.pollIntervalMs;

    String json;
    serializeJson(doc, json);

    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        return false;
    }
    bool ok = prefs.putString(NVS_KEY, json);
    prefs.end();
    return ok;
}

bool ConfigService::resetToDefaults(DeviceConfig &cfg) {
    cfg.wifiSsid = DEFAULT_WIFI_SSID;
    cfg.wifiPassword = DEFAULT_WIFI_PASSWORD;
    cfg.pollIntervalMs = HEARTBEAT_PERIOD_MS;
    return save(cfg);
}
