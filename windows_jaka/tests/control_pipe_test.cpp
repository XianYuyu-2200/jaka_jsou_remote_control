#include "control_pipe.hpp"

#include <cassert>
#include <cmath>

int main() {
    windows_jaka::ControlCommand command{};
    assert(windows_jaka::parse_control_line("JOG 4 -0.0015", command));
    assert(command.type == windows_jaka::ControlCommand::Type::Jog);
    assert(command.axis == 4);
    assert(std::abs(command.delta_rad + 0.0015) < 1e-12);

    command = {};
    assert(windows_jaka::parse_control_line("JOG_STOP 4", command));
    assert(command.type == windows_jaka::ControlCommand::Type::JogStop);

    command = {};
    assert(windows_jaka::parse_control_line("SAFEPOSE 0 0.1 -0.2 0.3 -0.4 0.5", command));
    assert(command.type == windows_jaka::ControlCommand::Type::SafetyPose);
    assert(std::abs(command.pose[5] - 0.5) < 1e-12);

    command = {};
    assert(windows_jaka::parse_control_line("DRAG 1", command));
    assert(command.type == windows_jaka::ControlCommand::Type::DragMode);
    assert(command.enabled);

    command = {};
    assert(windows_jaka::parse_control_line("DRAG 0", command));
    assert(command.type == windows_jaka::ControlCommand::Type::DragMode);
    assert(!command.enabled);

    assert(!windows_jaka::parse_control_line("DRAG 2", command));
    assert(!windows_jaka::parse_control_line("JOG 9 0.01", command));
    assert(!windows_jaka::parse_control_line("JOG 0 nan", command));
    assert(!windows_jaka::parse_control_line("JOG 0 inf", command));
    assert(!windows_jaka::parse_control_line("SAFEPOSE 0 0 0", command));
    assert(!windows_jaka::parse_control_line("SAFEPOSE 0 0 0 0 0 nan", command));
    return 0;
}
