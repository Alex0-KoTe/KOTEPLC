#ifndef FREERTOS_TASK_H
#define FREERTOS_TASK_H

#include <stddef.h>
#include <stdint.h>
#include <string>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================================
// Лёгкая обёртка над FreeRTOS: задача = экземпляр TaskBase с виртуальным run().
// Прикладные задачи наследуются от TaskBase и запускаются через start().
// Стек выделяется статически (xTaskCreateStatic) — детерминированно и без
// аллокации в куче во время работы планировщика.
// ============================================================================

namespace os {

class TaskBase {
public:
    struct Config {
        const char *name = "task";
        uint32_t stackBytes = 4096;
        UBaseType_t priority = tskIDLE_PRIORITY + 1;
        BaseType_t core = tskNO_AFFINITY; // 0 / 1 для привязки к ядру
    };

    virtual ~TaskBase() { stop(); }

    bool start(const Config &cfg) {
        cfg_ = cfg;
        stackBuf_.resize(cfg.stackBytes / sizeof(StackType_t));
        // Копия имени: cfg.name может указывать на временную строку
        nameBuf_ = cfg.name;
        // ВАЖНО: статический буфер задачи должен жить всё время работы задачи
        taskBufPtr_ = new (std::nothrow) StaticTask_t();
        if (!taskBufPtr_) return false;
        handle_ = xTaskCreateStaticPinnedToCore(
            &TaskBase::trampoline, nameBuf_.c_str(),
            static_cast<uint32_t>(stackBuf_.size()),
            this, cfg.priority,
            stackBuf_.data(), taskBufPtr_, cfg.core);
        if (!handle_) {
            delete taskBufPtr_;
            taskBufPtr_ = nullptr;
            return false;
        }
        running_ = true;
        return true;
    }

    void stop() {
        running_ = false;
        if (handle_) {
            vTaskDelete(handle_);
            handle_ = nullptr;
        }
    }

    bool isRunning() const { return handle_ != nullptr && running_; }

    static void delayMs(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

protected:
    virtual void run() = 0;   // тело задачи: цикл с периодикой внутри

private:
    static void trampoline(void *arg) {
        auto *self = static_cast<TaskBase *>(arg);
        self->run();
        self->running_ = false;
        self->handle_ = nullptr;
        vTaskDelete(nullptr); // завершение собственной задачи
    }

    Config cfg_{};
    std::string nameBuf_;
    TaskHandle_t handle_ = nullptr;
    volatile bool running_ = true;
    std::vector<StackType_t> stackBuf_;
    StaticTask_t *taskBufPtr_ = nullptr; // heap-аллокация на всё время жизни задачи
};

} // namespace os

#endif // FREERTOS_TASK_H
