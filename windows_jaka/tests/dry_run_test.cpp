#include "joint_mapping.hpp"

#include <cassert>
#include <cmath>

int main() {
    using namespace windows_jaka;
    JointLimiter limiter;
    JointArray zero{};
    limiter.reset(zero);
    OnePoleLowPass filter(0.5);
    filter.reset(zero);
    const JointArray mapped{0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
    const auto filtered = filter.filter(mapped);
    assert(filtered[0] > 0.0 && filtered[0] < mapped[0]);
    const auto command = limiter.step(filtered, 0.008);
    assert(command[0] >= 0.0 && command[0] < 0.01);
    assert(std::abs(command[1]) < 1e-12);

    // The default envelope must not introduce a multi-hundred-millisecond
    // acceleration ramp at the 8 ms servo period. Per-cycle slew limits still
    // apply, but a moderate target should be substantially approached within
    // 0.4 s.
    limiter.reset(zero);
    JointArray desired{0.2, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i < 50; ++i) limiter.step(desired, 0.008);
    assert(limiter.command()[0] > 0.15);
    for (int i = 0; i < 50; ++i) limiter.step(desired, 0.008);
    assert(limiter.command()[0] <= desired[0] + 1e-12);
    return 0;
}
