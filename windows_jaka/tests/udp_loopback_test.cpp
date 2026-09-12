#include "udp_transport.hpp"

#include <cassert>

int main() {
    using namespace windows_jaka;
    WinsockRuntime winsock;
    UdpSocket receiver;
    UdpSocket sender;
    receiver.open_receiver(39101);
    sender.open_sender("127.0.0.1", 39101);

    auto packet = make_empty_packet();
    packet.sequence = 42;
    packet.sdk_code = 0;
    packet.operator_valid = 1;
    packet.position[0] = 1.25;
    assert(sender.send_packet(packet));

    JointSamplePacket received{};
    const auto status = receiver.receive_packet_status(received, 1000);
    assert(status == UdpSocket::ReceiveStatus::Received);
    (void)status;
    assert(received.sequence == 42);
    assert(received.position[0] == 1.25);
    return 0;
}
