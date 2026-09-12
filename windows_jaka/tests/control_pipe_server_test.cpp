// Runtime test for the GUI <-> follower command channel.
//
// The GUI reconnects to the named pipe for every JOG/JOG_STOP line, so the
// server has to survive many sequential connect/disconnect cycles and keep the
// pipe name registered the whole time. Regression guard for the bug where the
// server recreated the pipe per command and dropped commands while the name was
// unregistered.
#include "control_pipe.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

namespace {

bool send_until_accepted(const std::wstring& name, const std::string& line, int attempts) {
    for (int i = 0; i < attempts; ++i) {
        if (windows_jaka::send_control_command(name, line)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

bool wait_for_type(windows_jaka::ControlMailbox& mailbox, windows_jaka::ControlCommand::Type type,
                   windows_jaka::ControlCommand& out, int attempts) {
    for (int i = 0; i < attempts; ++i) {
        windows_jaka::ControlCommand command{};
        if (mailbox.take(command) && command.type == type) {
            out = command;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
}

}  // namespace

int main() {
    const std::wstring name = L"\\\\.\\pipe\\jaka_dual_teleop_test";
    windows_jaka::ControlMailbox mailbox;
    std::atomic<bool> stopping{false};
    std::thread server([&] {
        windows_jaka::ControlPipeServer pipe(name, mailbox);
        pipe.run(stopping);
    });

    windows_jaka::ControlCommand command{};
    bool ok = true;

    // Several sequential connections, as the GUI produces while a jog button is
    // held down (one short-lived connection every 8 ms).
    for (int round = 0; round < 5 && ok; ++round) {
        ok = send_until_accepted(name, "JOG 3 0.002", 300);
        windows_jaka::ControlCommand jog{};
        ok = ok && wait_for_type(mailbox, windows_jaka::ControlCommand::Type::Jog, jog, 300);
        if (ok) {
            ok = jog.axis == 3 && std::abs(jog.delta_rad - 0.002) < 1e-12;
        }
    }

    if (ok) {
        ok = send_until_accepted(name, "JOG_STOP 3", 300);
        ok = ok && wait_for_type(mailbox, windows_jaka::ControlCommand::Type::JogStop, command, 300);
        ok = ok && command.axis == 3;
    }

    if (ok) {
        ok = send_until_accepted(name, "SAFEPOSE 0 0.1 -0.2 0.3 -0.4 0.5", 300);
        ok = ok && wait_for_type(mailbox, windows_jaka::ControlCommand::Type::SafetyPose, command, 300);
        ok = ok && std::abs(command.pose[0]) < 1e-12 && std::abs(command.pose[5] - 0.5) < 1e-12;
    }

    // Let the server loop observe the stop flag, then unblock its pending
    // ConnectNamedPipe with one final connection.
    stopping.store(true);
    windows_jaka::send_control_command(name, "STOP");
    server.join();

    if (!ok) {
        std::printf("control_pipe_server_test FAILED\n");
        return 1;
    }
    std::printf("control_pipe_server_test passed\n");
    return 0;
}
