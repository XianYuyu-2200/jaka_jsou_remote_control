#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace windows_jaka {

constexpr std::uint32_t kPacketMagic = 0x31414B4A;
constexpr std::uint16_t kPacketVersion = 2;
constexpr std::uint64_t kHoldAgeNs = 24'000'000ULL;
constexpr std::uint64_t kFaultAgeNs = 100'000'000ULL;

#pragma pack(push, 1)
struct JointSamplePacket {
    std::uint32_t magic;
    std::uint16_t version;
    std::uint16_t size;
    std::uint64_t sequence;
    std::uint64_t monotonic_ns;
    double position[6];
    std::int32_t sdk_code;
    std::uint8_t operator_powered;
    std::uint8_t operator_enabled;
    std::uint8_t operator_dragging;
    std::uint8_t operator_valid;
};
#pragma pack(pop)

static_assert(sizeof(JointSamplePacket) == 80, "wire packet must remain 80 bytes");

inline bool finite_position(const JointSamplePacket& packet) {
    for (double value : packet.position) {
        if (!std::isfinite(value)) return false;
    }
    return true;
}

inline bool valid_packet(const JointSamplePacket& packet) {
    return packet.magic == kPacketMagic &&
           packet.version == kPacketVersion &&
           packet.size == sizeof(JointSamplePacket) &&
           packet.sdk_code == 0 &&
           packet.operator_valid != 0 &&
           finite_position(packet);
}

inline JointSamplePacket make_empty_packet() {
    JointSamplePacket packet{};
    packet.magic = kPacketMagic;
    packet.version = kPacketVersion;
    packet.size = sizeof(JointSamplePacket);
    return packet;
}

}  // namespace windows_jaka
