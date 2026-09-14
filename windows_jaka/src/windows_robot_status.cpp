#include <windows.h>

#include "JAKAZuRobot.h"
#include "runtime_status.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_stop{false};

BOOL WINAPI console_handler(DWORD type) {
    if (type == CTRL_C_EVENT || type == CTRL_BREAK_EVENT ||
        type == CTRL_CLOSE_EVENT || type == CTRL_LOGOFF_EVENT ||
        type == CTRL_SHUTDOWN_EVENT) {
        g_stop.store(true);
        return TRUE;
    }
    return FALSE;
}

struct Options {
    std::string robot_id{"robot"};
    std::string ip;
    std::filesystem::path status_file;
    int interval_ms{500};
    double duration_sec{0.0};
};

int parse_interval(const std::string& value) {
    const int parsed = std::stoi(value);
    if (parsed < 100 || parsed > 60000) throw std::invalid_argument("interval must be 100..60000 ms");
    return parsed;
}

Options parse_options(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string argument(argv[i]);
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument(std::string("missing value for ") + name);
            return argv[++i];
        };
        if (argument == "--robot-id") options.robot_id = next("--robot-id");
        else if (argument == "--ip") options.ip = next("--ip");
        else if (argument == "--status-file") options.status_file = next("--status-file");
        else if (argument == "--interval-ms") options.interval_ms = parse_interval(next("--interval-ms"));
        else if (argument == "--duration-sec") options.duration_sec = std::stod(next("--duration-sec"));
        else if (argument == "--help" || argument == "-h") {
            std::cout << "windows_robot_status.exe --robot-id ID --ip IP --status-file PATH"
                         " [--interval-ms 500] [--duration-sec 0]\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown argument: " + argument);
        }
    }
    if (options.robot_id.empty()) throw std::invalid_argument("--robot-id is required");
    if (options.ip.empty()) throw std::invalid_argument("--ip is required");
    if (options.status_file.empty()) throw std::invalid_argument("--status-file is required");
    if (options.duration_sec < 0.0) throw std::invalid_argument("--duration-sec must be non-negative");
    return options;
}

std::wstring widen_ascii(const std::string& value) {
    return std::wstring(value.begin(), value.end());
}

std::wstring monitor_mutex_name(const std::string& robot_id) {
    std::wstring name = L"Local\\JakaRobotStatusMonitor_";
    for (unsigned char ch : robot_id) {
        name.push_back(std::isalnum(ch) ? static_cast<wchar_t>(ch) : L'_');
    }
    return name;
}

std::wstring monitor_stop_event_name(const std::string& robot_id) {
    std::wstring name = L"Local\\JakaRobotStatusStop_";
    for (unsigned char ch : robot_id) {
        name.push_back(std::isalnum(ch) ? static_cast<wchar_t>(ch) : L'_');
    }
    return name;
}

bool finite_joints(const JointValue& joints) {
    for (double value : joints.jVal) {
        if (!std::isfinite(value)) return false;
    }
    return true;
}

bool wait_for_stop_or_timeout(std::chrono::milliseconds duration, HANDLE stop_event) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (!g_stop.load() && std::chrono::steady_clock::now() < deadline) {
        if (stop_event && WaitForSingleObject(stop_event, 0) == WAIT_OBJECT_0) {
            g_stop.store(true);
            return false;
        }
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        std::this_thread::sleep_for(std::min(remaining, std::chrono::milliseconds(50)));
    }
    return !g_stop.load();
}

int run_monitor(const Options& options) {
    HANDLE mutex = CreateMutexW(nullptr, TRUE, monitor_mutex_name(options.robot_id).c_str());
    if (!mutex) return 4;
    if (GetLastError() == ERROR_ALREADY_EXISTS) {
        CloseHandle(mutex);
        return 0;
    }

    HANDLE stop_event = CreateEventW(nullptr, TRUE, FALSE,
                                     monitor_stop_event_name(options.robot_id).c_str());
    if (!stop_event) {
        CloseHandle(mutex);
        return 5;
    }

    JAKAZuRobot robot;
    bool logged_in = false;
    int login_ret = -1;
    std::uint64_t sequence = 0;
    std::uint64_t read_errors = 0;
    std::string write_error;

    auto publish = [&](const std::string& alarm, bool connected, bool powered,
                       bool enabled, bool dragging, bool valid, double rate_hz) {
        windows_jaka::RuntimeStatus status;
        status.robot_id = options.robot_id;
        status.mode = "monitor";
        status.alarm = alarm;
        status.connected = connected;
        status.powered = powered;
        status.enabled = enabled;
        status.dragging = dragging;
        status.valid = valid;
        status.servo = false;
        status.sequence = sequence++;
        status.read_errors = read_errors;
        status.login_code = login_ret;
        status.rate_hz = rate_hz;
        status.packet_age_ms = 0.0;
        std::string error;
        if (!windows_jaka::write_runtime_status(options.status_file, status, error) &&
            write_error.empty()) {
            write_error = error;
            std::cerr << "status write failed: " << error << "\n";
        }
    };

    publish("starting", false, false, false, false, false, 0.0);
    const auto started = std::chrono::steady_clock::now();
    const auto interval = std::chrono::milliseconds(options.interval_ms);

    while (!g_stop.load()) {
        if (options.duration_sec > 0.0) {
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - started).count();
            if (elapsed >= options.duration_sec) break;
        }

        if (!logged_in) {
            login_ret = robot.login_in(options.ip.c_str(), false);
            if (login_ret != 0) {
                publish("login failed", false, false, false, false, false, 0.0);
                if (!wait_for_stop_or_timeout(interval, stop_event)) break;
                continue;
            }
            logged_in = true;
        }

        RobotStatus_simple simple{};
        BOOL dragging = FALSE;
        JointValue joints{};
        const int status_ret = robot.get_robot_status_simple(&simple);
        const int drag_ret = robot.is_in_drag_mode(&dragging);
        const int joint_ret = robot.get_actual_joint_position(&joints);

        if (status_ret != 0 || drag_ret != 0) {
            ++read_errors;
            publish("status read failed", false, false, false, false, false, 0.0);
            if (logged_in) robot.login_out();
            logged_in = false;
            if (!wait_for_stop_or_timeout(interval, stop_event)) break;
            continue;
        }

        if (joint_ret != 0) ++read_errors;
        const bool valid = joint_ret == 0 && finite_joints(joints);
        publish(joint_ret == 0 ? "" : "joint read failed", true,
                simple.powered_on != 0, simple.enabled != 0, dragging != FALSE, valid,
                1000.0 / static_cast<double>(options.interval_ms));
        if (!wait_for_stop_or_timeout(interval, stop_event)) break;
    }

    if (logged_in) robot.login_out();
    publish("monitor stopped", false, false, false, false, false, 0.0);
    CloseHandle(stop_event);
    CloseHandle(mutex);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    std::cout.setf(std::ios::unitbuf);
    std::cerr.setf(std::ios::unitbuf);
    SetConsoleCtrlHandler(console_handler, TRUE);
    try {
        return run_monitor(parse_options(argc, argv));
    } catch (const std::exception& exception) {
        std::cerr << "windows_robot_status: " << exception.what() << "\n";
        return 2;
    }
}