#include "udp_transport.hpp"

#include <winsock2.h>
#include <ws2tcpip.h>

#include <stdexcept>
#include <string>

namespace windows_jaka {
namespace {

SOCKET as_socket(std::uintptr_t value) {
    return static_cast<SOCKET>(value);
}

void throw_wsa(const char* operation) {
    throw std::runtime_error(std::string(operation) + " failed, WSA error=" +
                             std::to_string(WSAGetLastError()));
}

sockaddr_in loopback_address(std::uint16_t port) {
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    return address;
}

}  // namespace

WinsockRuntime::WinsockRuntime() {
    WSADATA data{};
    const int result = WSAStartup(MAKEWORD(2, 2), &data);
    if (result != 0) {
        throw std::runtime_error("WSAStartup failed, error=" + std::to_string(result));
    }
}

WinsockRuntime::~WinsockRuntime() { WSACleanup(); }

UdpSocket::~UdpSocket() { close(); }

void UdpSocket::open_sender(const char* address, std::uint16_t port) {
    close();
    SOCKET socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket == INVALID_SOCKET) throw_wsa("socket");
    socket_ = static_cast<std::uintptr_t>(socket);
    opened_ = true;

    sockaddr_in destination{};
    destination.sin_family = AF_INET;
    destination.sin_port = htons(port);
    if (InetPtonA(AF_INET, address, &destination.sin_addr) != 1) {
        close();
        throw std::runtime_error(std::string("invalid IPv4 address: ") + address);
    }
    if (connect(socket, reinterpret_cast<const sockaddr*>(&destination), sizeof(destination)) == SOCKET_ERROR) {
        close();
        throw_wsa("connect UDP sender");
    }
}

void UdpSocket::open_receiver(std::uint16_t port) {
    close();
    SOCKET socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket == INVALID_SOCKET) throw_wsa("socket");
    socket_ = static_cast<std::uintptr_t>(socket);
    opened_ = true;
    BOOL reuse = TRUE;
    setsockopt(socket, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&reuse), sizeof(reuse));
    const sockaddr_in address = loopback_address(port);
    if (bind(socket, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == SOCKET_ERROR) {
        close();
        throw_wsa("bind UDP receiver");
    }
}

void UdpSocket::set_nonblocking(bool enabled) {
    if (!opened_) throw std::logic_error("UDP socket is not open");
    u_long mode = enabled ? 1UL : 0UL;
    if (ioctlsocket(as_socket(socket_), FIONBIO, &mode) == SOCKET_ERROR) {
        throw_wsa("ioctlsocket");
    }
}

bool UdpSocket::send_packet(const JointSamplePacket& packet) {
    if (!opened_) throw std::logic_error("UDP socket is not open");
    const int bytes = send(as_socket(socket_), reinterpret_cast<const char*>(&packet),
                           static_cast<int>(sizeof(packet)), 0);
    return bytes == static_cast<int>(sizeof(packet));
}

bool UdpSocket::receive_packet(JointSamplePacket& packet, int timeout_ms) {
    return receive_packet_status(packet, timeout_ms) == ReceiveStatus::Received;
}

UdpSocket::ReceiveStatus UdpSocket::receive_packet_status(JointSamplePacket& packet,
                                                          int timeout_ms) {
    if (!opened_) throw std::logic_error("UDP socket is not open");
    SOCKET socket = as_socket(socket_);
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(socket, &read_set);
    timeval timeout{};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    const int selected = select(0, &read_set, nullptr, nullptr, &timeout);
    if (selected == 0) return ReceiveStatus::Timeout;
    if (selected == SOCKET_ERROR) throw_wsa("select");
    const int bytes = recv(socket, reinterpret_cast<char*>(&packet),
                           static_cast<int>(sizeof(packet)), 0);
    if (bytes == SOCKET_ERROR) {
        const int error = WSAGetLastError();
        if (error == WSAEWOULDBLOCK) return ReceiveStatus::Timeout;
        throw std::runtime_error("recv UDP failed, WSA error=" + std::to_string(error));
    }
    return bytes == static_cast<int>(sizeof(packet))
               ? ReceiveStatus::Received
               : ReceiveStatus::InvalidDatagram;
}

void UdpSocket::close() {
    if (opened_) {
        closesocket(as_socket(socket_));
        opened_ = false;
        socket_ = static_cast<std::uintptr_t>(~0ULL);
    }
}

}  // namespace windows_jaka
