// Regression guard for the follower watchdog timestamp underflow.
//
// The follower loop samples `tick_ns` at the top of each iteration and later
// computes the age of the last processed command. A bug captured a fresh
// monotonic_ns() *after* tick_ns for the jog path, so `tick_ns - last` wrapped
// to ~2^64 and tripped a bogus WATCHDOG_100MS on the very first jog press,
// killing the real-motion session. elapsed_since_ns must clamp instead of wrap.
#include "joint_sample_packet.hpp"
#include "windows_clock.hpp"

#include <cstdint>
#include <cstdio>

int main() {
    bool ok = true;

    // Normal forward progress.
    ok = ok && windows_jaka::elapsed_since_ns(1000, 400) == 600;

    // Identical timestamps: no time has passed.
    ok = ok && windows_jaka::elapsed_since_ns(400, 400) == 0;

    // `then` in the future (out-of-order capture): must clamp to zero, never
    // wrap to a huge age. This is the exact shape of the original bug.
    const std::uint64_t tick_ns = 400;
    const std::uint64_t captured_later_ns = 425;
    ok = ok && windows_jaka::elapsed_since_ns(tick_ns, captured_later_ns) == 0;

    // The naive subtraction the bug used wraps to ~2^64, i.e. far past any
    // watchdog threshold, which is why it looked like a 100 ms stall.
    const std::uint64_t naive = tick_ns - captured_later_ns;
    ok = ok && naive > windows_jaka::kFaultAgeNs;

    if (!ok) {
        std::printf("windows_clock_test FAILED\n");
        return 1;
    }
    std::printf("windows_clock_test passed\n");
    return 0;
}