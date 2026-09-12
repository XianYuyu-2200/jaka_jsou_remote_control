#pragma once

#include "joint_sample_packet.hpp"

#include <cstdint>

namespace windows_jaka {

class WinsockRuntime {
public:
    WinsockRuntime();
    ~WinsockRuntime();
    WinsockRuntime(const WinsockRuntime&) = delete;
    WinsockRuntime& operator=(const WinsockRuntime&) = delete;
};

class UdpSocket {
public:
    enum class ReceiveStatus { Timeout, Received, InvalidDatagram };
    UdpSocket() = default;
    ~UdpSocket();
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    void open_sender(const char* address, std::uint16_t port);
    void open_receiver(std::uint16_t port);
    void set_nonblocking(bool enabled);
    bool send_packet(const JointSamplePacket& packet);
    bool receive_packet(JointSamplePacket& packet, int timeout_ms);
    ReceiveStatus receive_packet_status(JointSamplePacket& packet, int timeout_ms);
    void close();

private:
    std::uintptr_t socket_{static_cast<std::uintptr_t>(~0ULL)};
    bool opened_{false};
};

}  // namespace windows_jaka
