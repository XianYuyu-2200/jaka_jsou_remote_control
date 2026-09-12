#include "runtime_status.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

int main() {
    windows_jaka::RuntimeStatus status;
    status.robot_id = "follower_test";
    status.mode = "idle";
    status.connected = true;
    status.powered = true;
    status.enabled = true;
    status.dragging = false;
    status.valid = true;
    status.servo = false;
    status.sequence = 123;
    status.rate_hz = 99.5;
    status.packet_age_ms = 10.25;
    const auto path = std::filesystem::temp_directory_path() / "jaka_runtime_status_test.status";
    std::string error;
    if (!windows_jaka::write_runtime_status(path, status, error)) {
        std::cerr << error << "\n";
        return 1;
    }
    std::ifstream input(path);
    const std::string content((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    if (content.find("robot_id=follower_test") == std::string::npos ||
        content.find("connected=1") == std::string::npos ||
        content.find("sequence=123") == std::string::npos ||
        content.find("packet_age_ms=10.250") == std::string::npos) {
        std::cerr << content << "\n";
        return 2;
    }
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    return 0;
}
