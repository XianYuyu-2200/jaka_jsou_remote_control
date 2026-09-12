#include "udp_transport.hpp"


int main() {
    using namespace windows_jaka;
    WinsockRuntime winsock;
    UdpSocket receiver_a;
    UdpSocket receiver_b;
    UdpSocket sender;
    receiver_a.open_receiver(39101);
    receiver_b.open_receiver(39102);
    sender.open_sender_multi({
        UdpEndpoint{"127.0.0.1", 39101},
        UdpEndpoint{"127.0.0.1", 39102},
    });

    auto packet = make_empty_packet();
    packet.sequence = 42;
    packet.sdk_code = 0;
    packet.operator_valid = 1;
    packet.position[0] = 1.25;
    if (!sender.send_packet(packet)) return 1;

    JointSamplePacket received_a{};
    JointSamplePacket received_b{};
    const auto status_a = receiver_a.receive_packet_status(received_a, 1000);
    const auto status_b = receiver_b.receive_packet_status(received_b, 1000);
    if (status_a != UdpSocket::ReceiveStatus::Received) return 2;
    if (status_b != UdpSocket::ReceiveStatus::Received) return 3;
    if (received_a.sequence != 42 || received_b.sequence != 42) return 4;
    if (received_a.position[0] != 1.25 || received_b.position[0] != 1.25) return 5;
    return 0;
}
