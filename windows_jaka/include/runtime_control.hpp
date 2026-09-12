#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>

namespace windows_jaka {

class StopController {
public:
    void request_stop() {
        stopped_.store(true);
        condition_.notify_all();
    }

    bool stopping() const { return stopped_.load(); }

    template <class Rep, class Period>
    void wait_for(const std::chrono::duration<Rep, Period>& duration) {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_for(lock, duration, [this] { return stopped_.load(); });
    }

private:
    std::atomic<bool> stopped_{false};
    std::mutex mutex_;
    std::condition_variable condition_;
};

}  // namespace windows_jaka
