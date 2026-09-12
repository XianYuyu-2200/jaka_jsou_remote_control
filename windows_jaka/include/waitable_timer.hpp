#pragma once

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <stdexcept>

namespace windows_jaka {

class PeriodicWaitableTimer {
public:
    explicit PeriodicWaitableTimer(long period_ms) {
        timer_ = CreateWaitableTimerW(nullptr, FALSE, nullptr);
        if (timer_ == nullptr) throw std::runtime_error("CreateWaitableTimer failed");
        LARGE_INTEGER due{};
        due.QuadPart = -static_cast<LONGLONG>(period_ms) * 10'000LL;
        if (!SetWaitableTimer(timer_, &due, period_ms, nullptr, nullptr, FALSE)) {
            CloseHandle(timer_);
            timer_ = nullptr;
            throw std::runtime_error("SetWaitableTimer failed");
        }
    }

    ~PeriodicWaitableTimer() {
        if (timer_ != nullptr) CloseHandle(timer_);
    }

    PeriodicWaitableTimer(const PeriodicWaitableTimer&) = delete;
    PeriodicWaitableTimer& operator=(const PeriodicWaitableTimer&) = delete;

    void wait() {
        const DWORD result = WaitForSingleObject(timer_, INFINITE);
        if (result != WAIT_OBJECT_0) throw std::runtime_error("waitable timer wait failed");
    }

private:
    HANDLE timer_{nullptr};
};

}  // namespace windows_jaka
