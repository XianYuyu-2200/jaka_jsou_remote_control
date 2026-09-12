#pragma once

#include "joint_sample_packet.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace windows_jaka {

class WinsockRuntime {
public:
    WinsockRuntime();
    ~WinsockRuntime();
    WinsockRuntime(const WinsockRuntime&) = delete;
    WinsockRuntime& operator=(const WinsockRuntime&) = delete;
};

struct UdpEndpoint {
    std::string address;
    std::uint16_t port;
};

class UdpSocket {
public:
    enum class ReceiveStatus { Timeout, Received, InvalidDatagram };
    UdpSocket() = default;
    ~UdpSocket();
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    void open_sender(const char* address, std::uint16_t port);
    void open_sender_multi(const std::vector<UdpEndpoint>& endpoints);
    void open_receiver(std::uint16_t port);
    void set_nonblocking(bool enabled);
    bool send_packet(const JointSamplePacket& packet);
    bool receive_packet(JointSamplePacket& packet, int timeout_ms);
    ReceiveStatus receive_packet_status(JointSamplePacket& packet, int timeout_ms);
    void close();

private:
    std::uintptr_t socket_{static_cast<std::uintptr_t>(~0ULL)};
    bool opened_{false};
    struct Destination {
        std::uint32_t address;
        std::uint16_t port;
    };
    std::vector<Destination> destinations_;
};

}  // namespace windows_jaka
