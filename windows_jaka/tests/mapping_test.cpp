#include "joint_mapping.hpp"

#include <cassert>
#include <cmath>

int main() {
    using namespace windows_jaka;
    const JointArray operator_zero{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    const JointArray follower_zero{-0.1, -0.2, -0.3, -0.4, -0.5, -0.6};
    const auto target = map_relative(operator_zero, operator_zero, follower_zero);
    for (std::size_t i = 0; i < 6; ++i) assert(std::abs(target[i] - follower_zero[i]) < 1e-12);

    const JointArray operator_position{0.2, 0.1, 0.5, 0.2, 0.7, 0.4};
    const auto moved = map_relative(operator_position, operator_zero, follower_zero);
    assert(std::abs(moved[0] - 0.0) < 1e-12);
    assert(std::abs(moved[1] - (-0.3)) < 1e-12);
    return 0;
}
