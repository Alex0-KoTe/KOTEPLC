#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <stdint.h>

// ============================================================================
// Встроенный веб-интерфейс KOTELPC (ESPAsyncWebServer, порт 80).
//   GET  /            — SPA (тёмная тема, вкладки: сеть / состояние / файлы)
//   GET  /api/status  — JSON: Wi-Fi, DI/DO, heap, fw, FS
//   POST /api/network — сохранение настроек Wi-Fi {ssid,pass} + переподключение
//   Файлы LittleFS:
//     GET  /api/fs/list          — список /www
//     GET  /api/fs/file?path=..  — содержимое файла (только /www)
//     POST /api/fs/upload        — multipart-загрузка в /www
//     POST /api/fs/delete?path=  — удаление файла из /www
// ============================================================================

namespace web {

void start();                        // поднять сервер (вызывать после Wi-Fi begin)
bool isRunning();                    // сервер запущен
void setDiDo(uint8_t di, uint8_t do_, bool mcpReady);  // снимок состояний из AppLogicTask

} // namespace web

#endif // WEB_SERVER_H
