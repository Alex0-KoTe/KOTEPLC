// Драйвер LittleFS поверх разделов ESP-IDF (esp_partition + esp_vfs_littlefs).
// Требует раздела с именем "storage" (см. partitions.csv в корне проекта).

#include "fs_driver.h"

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <esp_littlefs.h>
#include <esp_partition.h>
#include <cerrno>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <cstdio>
#include <esp_log.h>

static const char *TAG = "LITTLEFS";
static const char *MOUNT_POINT = "/littlefs";

namespace fs {

namespace {
bool lock(void *&m) {
    if (!m) m = xSemaphoreCreateMutex();
    return m && xSemaphoreTake((SemaphoreHandle_t)m, pdMS_TO_TICKS(2000)) == pdTRUE;
}
void unlock(void *m) { if (m) xSemaphoreGive((SemaphoreHandle_t)m); }

std::string fullPath(const std::string &p) {
    if (!p.empty() && p[0] == '/') return MOUNT_POINT + p;
    return std::string(MOUNT_POINT) + "/" + p;
}
} // namespace

FsDriver &FsDriver::instance() {
    static FsDriver inst;
    return inst;
}

bool FsDriver::begin(bool allowFormat) {
    if (mounted_) return true;

    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    if (!part) {
        ESP_LOGE(TAG, "partition 'storage' not found (check partitions.csv)");
        return false;
    }

    esp_vfs_littlefs_conf_t conf = {};
    conf.base_path = MOUNT_POINT;
    conf.partition = part;
    conf.format_if_mount_failed = allowFormat;
    conf.grow_on_mount = true;

    esp_err_t err = esp_vfs_littlefs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mount failed: %s", esp_err_to_name(err));
        return false;
    }

    mounted_ = true;
    size_t total = 0, used = 0;
    // esp_littlefs_info() в esp_littlefs v1.14.8 принимает label раздела (строку)
    if (esp_littlefs_info("storage", &total, &used) == ESP_OK) {
        ESP_LOGI(TAG, "mounted '%s': %zu/%zu bytes used", part->label, used, total);
    } else {
        ESP_LOGI(TAG, "mounted '%s'", part->label);
    }
    return true;
}

bool FsDriver::readFile(const std::string &path, std::string &out) {
    if (!lock(mutex_)) return false;
    FILE *f = fopen(fullPath(path).c_str(), "rb");
    if (!f) { unlock(mutex_); return false; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    out.assign((size_t)(sz > 0 ? sz : 0), '\0');
    size_t rd = sz > 0 ? fread(out.data(), 1, out.size(), f) : 0;
    fclose(f);
    unlock(mutex_);
    if (rd != out.size()) { out.clear(); return false; }
    return true;
}

bool FsDriver::writeFile(const std::string &path, const std::string &data) {
    if (!lock(mutex_)) return false;
    FILE *f = fopen(fullPath(path).c_str(), "wb");
    if (!f) { unlock(mutex_); return false; }
    size_t wr = data.empty() ? 0 : fwrite(data.data(), 1, data.size(), f);
    fflush(f);
    fclose(f);
    unlock(mutex_);
    return wr == data.size();
}

bool FsDriver::appendFile(const std::string &path, const std::string &data) {
    if (!lock(mutex_)) return false;
    FILE *f = fopen(fullPath(path).c_str(), "ab");
    if (!f) { unlock(mutex_); return false; }
    size_t wr = data.empty() ? 0 : fwrite(data.data(), 1, data.size(), f);
    fflush(f);
    fclose(f);
    unlock(mutex_);
    return wr == data.size();
}

bool FsDriver::exists(const std::string &path) {
    struct stat st;
    return ::stat(fullPath(path).c_str(), &st) == 0;
}

bool FsDriver::remove(const std::string &path) {
    if (!lock(mutex_)) return false;
    int rc = unlink(fullPath(path).c_str());
    unlock(mutex_);
    return rc == 0;
}

bool FsDriver::mkdir(const std::string &path) {
    if (!lock(mutex_)) return false;
    int rc = ::mkdir(fullPath(path).c_str(), 0775);
    unlock(mutex_);
    return rc == 0 || errno == EEXIST;
}

bool FsDriver::stat(const std::string &path, FileStats &out) {
    struct stat st;
    if (::stat(fullPath(path).c_str(), &st) != 0) return false;
    out.size = st.st_size;
    out.isDir = S_ISDIR(st.st_mode);
    return true;
}

bool FsDriver::listDir(const std::string &path,
                       std::function<void(const std::string &, bool)> cb) {
    DIR *d = opendir(fullPath(path).c_str());
    if (!d) return false;
    struct dirent *e;
    while ((e = readdir(d)) != nullptr) {
        if (e->d_name[0] == '.' && e->d_name[1] == '\0') continue;
        cb(e->d_name, e->d_type == DT_DIR);
    }
    closedir(d);
    return true;
}

bool FsDriver::spaceInfo(size_t &total, size_t &used) {
    // API esp_littlefs v1.14.8: esp_littlefs_info(const char* partition_label, ...)
    return esp_littlefs_info("storage", &total, &used) == ESP_OK;
}

} // namespace fs
