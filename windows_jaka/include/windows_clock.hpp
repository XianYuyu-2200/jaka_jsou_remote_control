#pragma once

#include <chrono>
#include <cstdint>

namespace windows_jaka {

// Elapsed nanoseconds from `then` to `now`, clamped at zero.
//
// Timestamps captured out of order (for example a fresh clock read taken after
// the loop's tick timestamp) used to wrap this unsigned subtraction to ~2^64
// and trip a bogus WATCHDOG_100MS. Always clamp so a `then` in the future reads
// as zero elapsed time instead of a huge age.
inline std::uint64_t elapsed_since_ns(std::uint64_t now_ns, std::uint64_t then_ns) {
    return now_ns > then_ns ? now_ns - then_ns : 0;
}

inline std::uint64_t monotonic_ns() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

}  // namespace windows_jaka