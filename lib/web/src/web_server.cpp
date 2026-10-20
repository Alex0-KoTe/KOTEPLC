// Веб-интерфейс KOTELPC: REST API + одностраничный UI (тёмная тема).
// Сервер — ESPAsyncWebServer (порт 80), работает на стеке AsyncTCP,
// поэтому блокирующих операций в обработчиках нет; тяжёлые действия
// (сохранение настроек Wi-Fi) откладываются в отдельную FreeRTOS-задачу.
//
// FW 0.6.8 (крах tcp_alloc / lwIP): все сетевые операции — WiFi.mode/softAP,
// TCP listen/connect и т.п. — выполняются ТОЛЬКО из задачи loopTask (ядро 1,
// контекст Arduino). Это обязательное требование esp_wifi/lwIP-стека Arduino:
// вызовы из других задач гоняют wifi/lwIP-мьютексы между ядрами и приводят к
// краху "assert failed: tcp_alloc ... Required to lock TCPIP core functionality".
// Обработчик POST /api/network теперь лишь публикует запрос; применяет его
// netApplyTask через tickLoop() (см. ниже), который передаёт управление
// loopTask'у и ждёт флага s_netDone от handleLoopTick().
#include "web_server.h"

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <string.h>
#include <stdio.h>

#include "project_config.h"
#include "hal/hal_uart_bridge.h"   // LOGP
#include "wifi_manager.h"
#include "config_service.h"
#include "fs_driver.h"
#include "index_html.h"            // INDEX_HTML (PROGMEM)

// ---------- снимок состояний DI/DO (пишет AppLogicTask, читает /api/status) ----------
static volatile uint8_t s_di = 0;
static volatile uint8_t s_do = 0;
static volatile bool s_mcpReady = false;
static bool s_started = false;

// ---------- запрос на применение сетевых настроек (из POST /api/network) ----------
struct NetRequest { char ssid[65]; char pass[65]; };
static volatile bool s_netPending = false;
static volatile bool s_netDone = false;   // loopTask подтверждает выполнение
static NetRequest s_netReq;
static SemaphoreHandle_t s_netMutex = nullptr;

static AsyncWebServer s_server(80);

// Путь должен лежать внутри /www и не содержать "..".
static bool safeWwwPath(const String &raw, std::string &out) {
    if (!raw.startsWith("/www")) return false;
    if (raw.indexOf("..") >= 0) return false;
    out = raw.c_str();
    return out.size() > 4 && out.size() < 96;
}

// Простой JSON-ответ (для ok/error).
static void replyJson(AsyncWebServerRequest *req, int code, const char *json) {
    req->send(code, "application/json", String(json));
}

// ---------------- GET /api/status ----------------
static void handleStatus(AsyncWebServerRequest *req) {
    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();
    root["fw"] = FW_VERSION;
    root["uptime_s"] = (uint32_t)(millis() / 1000);
    root["heap_free"] = (uint32_t)ESP.getFreeHeap();

    JsonObject wifi = root["wifi"].to<JsonObject>();
    // Режим этой платы — AP+STA: точка доступа всегда активна, STA сверху.
    wifi["mode"] = g_wifi.isConnected() ? "ap+sta" : "ap";
    wifi["connected"] = g_wifi.isConnected();
    wifi["ssid"] = g_wifi.currentSsid();
    wifi["ip"] = g_wifi.localIP();
    wifi["rssi"] = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;

    JsonObject io = root["io"].to<JsonObject>();
    io["mcp_ready"] = (bool)s_mcpReady;
    io["di"] = (uint32_t)s_di;   // бит i = 1 => DIi активен (дебаунсенно)
    io["do"] = (uint32_t)s_do;   // бит i = 1 => реле RLi включено

    JsonObject fsys = root["fs"].to<JsonObject>();
    size_t total = 0, used = 0;
    if (fs::FsDriver::instance().spaceInfo(total, used)) {
        fsys["ok"] = true; fsys["total"] = (uint32_t)total; fsys["used"] = (uint32_t)used;
    } else {
        fsys["ok"] = false;
    }

    String body;
    serializeJson(doc, body);
    req->send(200, "application/json", body);
}

// ---------------- POST /api/network?ssid=...&pass=... ----------------
// Параметры передаются через query-string (POST-аргументы): в ESP Async WebServer
// 1.2.x приём тела требует onRequestBody с return true, что на практике ломало
// обработку. Браузер URL-кодирует значения сам (URLSearchParams), поэтому спец-
// символы (&, =, пробел, кириллица в SSID) передаются корректно.
static void handleNetworkPost(AsyncWebServerRequest *req) {
    if (!req->hasParam("ssid", true)) {
        replyJson(req, 400, "{\"ok\":false,\"error\":\"param 'ssid' missing\"}");
        return;
    }
    String ssid = req->getParam("ssid", true)->value();
    String pass = req->hasParam("pass", true) ? req->getParam("pass", true)->value() : String("");
    if (ssid.length() == 0 || ssid.length() > 64 || pass.length() > 64) {
        replyJson(req, 400, "{\"ok\":false,\"error\":\"ssid too long (max 64)\"}");
        return;
    }
    xSemaphoreTake(s_netMutex, portMAX_DELAY);
    memset((void *)&s_netReq, 0, sizeof(s_netReq));
    strncpy(s_netReq.ssid, ssid.c_str(), sizeof(s_netReq.ssid) - 1);
    strncpy(s_netReq.pass, pass.c_str(), sizeof(s_netReq.pass) - 1);
    s_netPending = true;
    xSemaphoreGive(s_netMutex);
    replyJson(req, 200, "{\"ok\":true}");
}

// Задача применения сетевых настроек (netApplyTask) удалена в 0.6.11: она не
// трогала Wi-Fi-стек (FW 0.6.8 перенёс всю логику в web::handleLoopTick(),
// контекст loopTask), но бесконечная задача-пустышка с vTaskDelay(500) при
// приоритете tskIDLE_PRIORITY+2 отнимала квант планировщика у loopTask и
// попадала в дампы uxTaskGetSystemState, запутывая диагностику boot loop.

// ---------------- файлы LittleFS (/www) ----------------
static void handleFsList(AsyncWebServerRequest *req) {
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    fs::FsDriver::instance().listDir("/www", [&](const std::string &name, bool isDir) {
        if (isDir) return;
        JsonObject f = arr.add<JsonObject>();
        f["name"] = name;
        fs::FileStats st{};
        std::string full = "/www/" + name;
        if (fs::FsDriver::instance().stat(full, st)) f["size"] = (uint32_t)st.size;
        else f["size"] = 0;
    });
    String body;
    serializeJson(doc, body);
    req->send(200, "application/json", body);
}

static void handleFsFile(AsyncWebServerRequest *req) {
    if (!req->hasParam("path")) {
        req->send(400, "text/plain", "param 'path' missing"); return;
    }
    std::string path;
    if (!safeWwwPath(req->getParam("path")->value(), path)) {
        req->send(403, "text/plain", "forbidden"); return;
    }
    std::string data;
    if (!fs::FsDriver::instance().readFile(path, data)) {
        req->send(404, "text/plain", "not found"); return;
    }
    req->send(200, "text/plain; charset=utf-8", String(data.c_str()));
}

static void handleFsDelete(AsyncWebServerRequest *req) {
    std::string path;
    if (!req->hasParam("path") || !safeWwwPath(req->getParam("path")->value(), path)) {
        req->send(403, "text/plain", "forbidden"); return;
    }
    bool ok = fs::FsDriver::instance().remove(path);
    replyJson(req, ok ? 200 : 404, ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

// Multipart upload в /www (<input name="file" filename=...>).
// Обработчик вызывается из контекста AsyncTCP; fwrite идёт в VFS LittleFS.
struct UploadCtx { FILE *f; std::string path; bool bad; };

static void handleUpload(AsyncWebServerRequest *req,
                         const String &filename, size_t index,
                         const uint8_t *data, size_t len, bool final) {
    static UploadCtx ctx = {nullptr, "", false};

    if (index == 0) {   // начало файла
        if (ctx.f) { fclose(ctx.f); ctx.f = nullptr; }
        ctx.bad = true;
        String base = filename.substring(filename.lastIndexOf('/') + 1);
        if (base.length() > 0 && base.indexOf("..") < 0 && base.length() <= 32) {
            ctx.path = std::string("/littlefs/www/") + base.c_str();
            fs::FsDriver::instance().mkdir("/www");
            ctx.f = fopen(ctx.path.c_str(), "wb");
            if (ctx.f) ctx.bad = false;
        }
    }
    if (ctx.f && len) fwrite(data, 1, len, ctx.f);

    if (final) {
        if (ctx.f) { fclose(ctx.f); ctx.f = nullptr; }
        replyJson(req, ctx.bad ? 500 : 200,
                  ctx.bad ? "{\"ok\":false,\"error\":\"open failed\"}" : "{\"ok\":true}");
    }
}

// ---------------- главная страница ----------------
static void handleIndex(AsyncWebServerRequest *req) {
    req->send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

// ---------- запуск / состояние ----------
namespace web {

void setDiDo(uint8_t di, uint8_t do_, bool mcpReady) {
    s_di = di; s_do = do_; s_mcpReady = mcpReady;
}

bool isRunning() { return s_started; }

// FW 0.6.8: применение сетевых настроек в контексте loopTask (см. main.cpp).
// Вызывается из loop(); выполняет NVS-сохранение и переподключение STA здесь,
// а НЕ в netApplyTask — все WiFi.begin()/mode()/softAP обязаны идти из одной
// задачи (loopTask, ядро Arduino), иначе lwIP/esp_wifi падают по assert.
void handleLoopTick() {
    if (!s_netPending) return;
    NetRequest rq;
    xSemaphoreTake(s_netMutex, portMAX_DELAY);
    rq = s_netReq;
    s_netPending = false;
    xSemaphoreGive(s_netMutex);

    LOGP("[WEB] applying network (loopTask ctx): ssid=\"%s\"\n", rq.ssid);
    ConfigService svc;
    DeviceConfig cfg;
    svc.load(cfg);
    cfg.wifiSsid = rq.ssid;
    cfg.wifiPassword = rq.pass;
    svc.save(cfg);
    g_wifi.begin(rq.ssid, rq.pass);      // AP поднят всегда; сверху — STA
    LOGP("[WEB] wifi now: sta=%s ip=%s\n",
         g_wifi.isConnected() ? "connected" : "down",
         g_wifi.localIP().c_str());
    s_netDone = true;
}

void start() {
    if (s_started) return;
    if (!s_netMutex) s_netMutex = xSemaphoreCreateMutex();

    s_server.on("/", HTTP_GET, handleIndex);
    s_server.on("/api/status", HTTP_GET, handleStatus);
    s_server.on("/api/network", HTTP_POST, handleNetworkPost);
    s_server.on("/api/fs/list", HTTP_GET, handleFsList);
    s_server.on("/api/fs/file", HTTP_GET, handleFsFile);
    s_server.on("/api/fs/delete", HTTP_POST, handleFsDelete);
    s_server.on("/api/fs/upload", HTTP_POST,
                [](AsyncWebServerRequest *req) {
                    replyJson(req, 400, "{\"ok\":false,\"error\":\"no file part\"}");
                },
                handleUpload);
    s_server.begin();
    s_started = true;
    // 0.6.11: netApplyTask (пустышка, см. выше) больше не создаётся.
    LOGP("[WEB] server started on :80\n");
}

} // namespace web
