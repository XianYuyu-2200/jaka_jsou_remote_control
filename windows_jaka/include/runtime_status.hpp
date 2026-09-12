#pragma once

#include <windows.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace windows_jaka {

struct RuntimeStatus {
    std::string robot_id;
    std::string mode;
    std::string alarm;
    bool connected{false};
    bool powered{false};
    bool enabled{false};
    bool dragging{false};
    bool valid{false};
    bool servo{false};
    std::uint64_t sequence{0};
    std::uint64_t watchdog_ticks{0};
    std::uint64_t dropped_packets{0};
    std::uint64_t read_errors{0};
    std::uint64_t send_errors{0};
    int login_code{-1};
    int servo_error{0};
    double rate_hz{0.0};
    double packet_age_ms{0.0};
    double packet_loss_percent{0.0};
};

inline std::string escape_status_value(const std::string& value) {
    std::string output;
    output.reserve(value.size());
    for (char ch : value) {
        if (ch == '\r' || ch == '\n') output.push_back(' ');
        else output.push_back(ch);
    }
    return output;
}

inline bool write_runtime_status(const std::filesystem::path& path,
                                 const RuntimeStatus& status,
                                 std::string& error) {
    error.clear();
    std::error_code directory_error;
    std::filesystem::create_directories(path.parent_path(), directory_error);
    if (directory_error) {
        error = "cannot create status directory: " + directory_error.message();
        return false;
    }

    std::filesystem::path temporary = path;
    temporary += L".tmp";
    std::ofstream output(temporary, std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        error = "cannot write status file: " + temporary.string();
        return false;
    }
    output << "robot_id=" << escape_status_value(status.robot_id) << '\n';
    output << "mode=" << escape_status_value(status.mode) << '\n';
    output << "connected=" << (status.connected ? 1 : 0) << '\n';
    output << "powered=" << (status.powered ? 1 : 0) << '\n';
    output << "enabled=" << (status.enabled ? 1 : 0) << '\n';
    output << "dragging=" << (status.dragging ? 1 : 0) << '\n';
    output << "valid=" << (status.valid ? 1 : 0) << '\n';
    output << "servo=" << (status.servo ? 1 : 0) << '\n';
    output << "sequence=" << status.sequence << '\n';
    output << "watchdog_ticks=" << status.watchdog_ticks << '\n';
    output << "dropped_packets=" << status.dropped_packets << '\n';
    output << "read_errors=" << status.read_errors << '\n';
    output << "send_errors=" << status.send_errors << '\n';
    output << "login_code=" << status.login_code << '\n';
    output << "servo_error=" << status.servo_error << '\n';
    output << std::fixed << std::setprecision(3);
    output << "rate_hz=" << status.rate_hz << '\n';
    output << "packet_age_ms=" << status.packet_age_ms << '\n';
    output << "packet_loss_percent=" << status.packet_loss_percent << '\n';
    output << "alarm=" << escape_status_value(status.alarm) << '\n';
    output.flush();
    output.close();
    if (!output) {
        error = "failed writing status file: " + temporary.string();
        return false;
    }

    if (!MoveFileExW(temporary.c_str(), path.c_str(),
                     MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
        error = "cannot replace status file: " + path.string();
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        return false;
    }
    return true;
}

}  // namespace windows_jaka
