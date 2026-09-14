#pragma once

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "joint_mapping.hpp"

#include <array>
#include <atomic>
#include <mutex>
#include <sstream>
#include <string>

namespace windows_jaka {

struct ControlCommand {
    enum class Type { None, Jog, JogStop, SafetyPose, DragMode, Stop };
    Type type{Type::None};
    int axis{-1};
    double delta_rad{0.0};
    JointArray pose{};
    bool enabled{false};
};

class ControlMailbox {
public:
    void publish(const ControlCommand& command) {
        std::lock_guard<std::mutex> lock(mutex_);
        command_ = command;
        pending_ = true;
    }

    bool take(ControlCommand& command) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!pending_) return false;
        command = command_;
        pending_ = false;
        return true;
    }

private:
    std::mutex mutex_;
    ControlCommand command_{};
    bool pending_{false};
};

inline bool parse_control_line(const std::string& line, ControlCommand& command) {
    std::istringstream input(line);
    std::string kind;
    input >> kind;
    if (kind == "JOG") {
        command.type = ControlCommand::Type::Jog;
        return static_cast<bool>(input >> command.axis >> command.delta_rad) &&
               command.axis >= 0 && command.axis < 6 && std::isfinite(command.delta_rad);
    }
    if (kind == "JOG_STOP") {
        command.type = ControlCommand::Type::JogStop;
        return static_cast<bool>(input >> command.axis) && command.axis >= 0 && command.axis < 6;
    }
    if (kind == "SAFEPOSE") {
        command.type = ControlCommand::Type::SafetyPose;
        for (double& value : command.pose) {
            if (!(input >> value)) return false;
        }
        return all_finite(command.pose);
    }
    if (kind == "DRAG") {
        int enabled = 0;
        if (!(input >> enabled) || (enabled != 0 && enabled != 1)) return false;
        command.type = ControlCommand::Type::DragMode;
        command.enabled = enabled != 0;
        return true;
    }
    if (kind == "STOP") {
        command.type = ControlCommand::Type::Stop;
        return true;
    }
    return false;
}

class ControlPipeServer {
public:
    ControlPipeServer(const std::wstring& name, ControlMailbox& mailbox)
        : name_(name), mailbox_(mailbox) {}

    void run(std::atomic<bool>& stopping) {
        // Keep one handle alive for the whole run. Creating and closing the pipe
        // for every command left the name unregistered between connections, so
        // the GUI's WaitNamedPipe/CreateFile raced with it and spuriously
        // reported "cannot reach the follower command channel" while dropping
        // JOG/JOG_STOP lines. Disconnect + reconnect keeps the name registered.
        HANDLE pipe = CreateNamedPipeW(
            name_.c_str(), PIPE_ACCESS_INBOUND,
            PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT,
            PIPE_UNLIMITED_INSTANCES, 0, 4096, 100, nullptr);
        if (pipe == INVALID_HANDLE_VALUE) return;
        while (!stopping.load()) {
            const BOOL connected = ConnectNamedPipe(pipe, nullptr) || GetLastError() == ERROR_PIPE_CONNECTED;
            if (!connected) break;
            char buffer[4096]{};
            DWORD read = 0;
            if (ReadFile(pipe, buffer, sizeof(buffer) - 1, &read, nullptr) && read > 0) {
                buffer[read] = '\0';
                std::istringstream lines(std::string(buffer, read));
                std::string line;
                while (std::getline(lines, line)) {
                    ControlCommand command{};
                    if (parse_control_line(line, command)) mailbox_.publish(command);
                }
            }
            FlushFileBuffers(pipe);
            DisconnectNamedPipe(pipe);
        }
        CloseHandle(pipe);
    }

private:
    std::wstring name_;
    ControlMailbox& mailbox_;
};

inline bool send_control_command(const std::wstring& name, const std::string& line) {
    if (!WaitNamedPipeW(name.c_str(), 100)) return false;
    HANDLE pipe = CreateFileW(name.c_str(), GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (pipe == INVALID_HANDLE_VALUE) return false;
    DWORD written = 0;
    const std::string payload = line + "\n";
    const BOOL ok = WriteFile(pipe, payload.data(), static_cast<DWORD>(payload.size()), &written, nullptr);
    CloseHandle(pipe);
    return ok && written == payload.size();
}

}  // namespace windows_jaka
