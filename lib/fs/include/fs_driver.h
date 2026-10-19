#ifndef FS_DRIVER_H
#define FS_DRIVER_H

#include <stdint.h>
#include <stddef.h>
#include <functional>
#include <string>

// ============================================================================
// Драйвер файловой системы LittleFS поверх SPI Flash (раздел "storage").
// Интерфейс похож на POSIX: mount/open/read/write/close/remove.
// Раздел задаётся в platformio.ini: board_build.partitions = partitions.csv
// ============================================================================

namespace fs {

struct FileStats {
    size_t size;
    bool isDir;
};

class FsDriver {
public:
    static FsDriver &instance();

    // Инициализация и монтирование LittleFS. Форматирует при первом запуске
    // или при повреждении, если allowFormat=true.
    bool begin(bool allowFormat = true);
    bool mounted() const { return mounted_; }

    // --- Файловые операции (потокобезопасны через mutex FreeRTOS) ---
    bool readFile(const std::string &path, std::string &out);
    bool writeFile(const std::string &path, const std::string &data);
    bool appendFile(const std::string &path, const std::string &data);
    bool exists(const std::string &path);
    bool remove(const std::string &path);
    bool mkdir(const std::string &path);
    bool listDir(const std::string &path, std::function<void(const std::string &name, bool isDir)> cb);
    bool stat(const std::string &path, FileStats &out);

    // Свободное/занятое место в разделе (байты)
    bool spaceInfo(size_t &total, size_t &used);

private:
    FsDriver() = default;
    bool mounted_ = false;
    void *mutex_ = nullptr; // SemaphoreHandle_t (спрятан от заголовка)
};

} // namespace fs

#endif // FS_DRIVER_H
