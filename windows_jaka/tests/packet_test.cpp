#include "joint_sample_packet.hpp"

#include <cassert>
#include <cmath>

int main() {
    using namespace windows_jaka;
    static_assert(sizeof(JointSamplePacket) == 80);
    auto packet = make_empty_packet();
    packet.sdk_code = 0;
    packet.operator_valid = 1;
    for (int i = 0; i < 6; ++i) packet.position[i] = 0.1 * i;
    assert(valid_packet(packet));
    packet.position[2] = NAN;
    assert(!valid_packet(packet));
    return 0;
}
