#include <windows.h>

#include "JAKAZuRobot.h"
#include "control_pipe.hpp"
#include "joint_mapping.hpp"
#include "joint_sample_packet.hpp"
#include "runtime_control.hpp"
#include "trajectory.hpp"
#include "udp_transport.hpp"
#include "windows_clock.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

windows_jaka::StopController* g_stop = nullptr;

BOOL WINAPI console_handler(DWORD type) {
    if (type == CTRL_C_EVENT || type == CTRL_BREAK_EVENT ||
        type == CTRL_CLOSE_EVENT || type == CTRL_LOGOFF_EVENT ||
        type == CTRL_SHUTDOWN_EVENT) {
        if (g_stop != nullptr) g_stop->request_stop();
        return TRUE;
    }
    return FALSE;
}

struct Options {
    std::string operator_ip{"192.168.0.101"};
    std::uint16_t port{30001};
    std::vector<std::uint16_t> peer_ports;
    double duration_sec{0.0};
    std::string record_file;
    std::string playback_file;
    double playback_speed{1.0};
    bool arm_motion{false};
    std::wstring control_pipe{L"\\\\.\\pipe\\jaka_operator_teleop"};
    std::string control_mode{"idle"};
    std::string filter{"none"};
    double lpf_cutoff{2.5};
    double max_velocity{1.0};
    double max_acceleration{8.0};
};

std::uint16_t parse_port(const std::string& value) {
    const unsigned long parsed = std::stoul(value);
    if (parsed == 0 || parsed > 65535) throw std::invalid_argument("invalid UDP port");
    return static_cast<std::uint16_t>(parsed);
}

Options parse_options(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument(std::string("missing value for ") + name);
            return argv[++i];
        };
        if (arg == "--operator-ip") options.operator_ip = next("--operator-ip");
        else if (arg == "--port") options.port = parse_port(next("--port"));
        else if (arg == "--peer-port") options.peer_ports.push_back(parse_port(next("--peer-port")));
        else if (arg == "--duration-sec") options.duration_sec = std::stod(next("--duration-sec"));
        else if (arg == "--record-file") options.record_file = next("--record-file");
        else if (arg == "--playback-file") options.playback_file = next("--playback-file");
        else if (arg == "--playback-speed") options.playback_speed = std::stod(next("--playback-speed"));
        else if (arg == "--arm-motion") options.arm_motion = true;
        else if (arg == "--dry-run") options.arm_motion = false;
        else if (arg == "--control-pipe") {
            const std::string value = next("--control-pipe");
            options.control_pipe.assign(value.begin(), value.end());
        }
        else if (arg == "--control-mode") options.control_mode = next("--control-mode");
        else if (arg == "--filter") options.filter = next("--filter");
        else if (arg == "--lpf-cutoff") options.lpf_cutoff = std::stod(next("--lpf-cutoff"));
        else if (arg == "--max-velocity") options.max_velocity = std::stod(next("--max-velocity"));
        else if (arg == "--max-acceleration") options.max_acceleration = std::stod(next("--max-acceleration"));
        else if (arg == "--help" || arg == "-h") {
            std::cout << "windows_operator.exe [--operator-ip IP] [--port N] [--duration-sec S]"
                         " [--peer-port N ...]"
                         " [--record-file PATH] [--playback-file PATH] [--playback-speed 1.0]"
                         " [--dry-run|--arm-motion]"
                         " [--control-pipe \\\\.\\pipe\\jaka_operator_teleop]"
                         " [--control-mode idle|teleop|joint|record|playback]"
                         " [--filter none|lpf|nlf] [--lpf-cutoff 2.5]"
                         " [--max-velocity 1.0] [--max-acceleration 8.0]\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown argument: " + arg);
        }
    }
    if (!(options.duration_sec >= 0.0)) throw std::invalid_argument("duration must be non-negative");
    if (options.filter != "none" && options.filter != "lpf" && options.filter != "nlf") {
        throw std::invalid_argument("--filter must be none, lpf, or nlf");
    }
    if (!(options.lpf_cutoff > 0.0)) throw std::invalid_argument("--lpf-cutoff must be positive");
    if (!(options.max_velocity > 0.0)) throw std::invalid_argument("--max-velocity must be positive");
    if (!(options.max_acceleration > 0.0)) throw std::invalid_argument("--max-acceleration must be positive");
    if (!(options.playback_speed > 0.0 && options.playback_speed <= 4.0)) {
        throw std::invalid_argument("--playback-speed must be in (0, 4]");
    }
    if (options.control_mode != "idle" && options.control_mode != "teleop" &&
        options.control_mode != "joint" && options.control_mode != "record" &&
        options.control_mode != "playback") {
        throw std::invalid_argument("--control-mode must be idle, teleop, joint, record, or playback");
    }
    if (options.control_mode == "record" && options.record_file.empty()) {
        throw std::invalid_argument("--control-mode record requires --record-file");
    }
    if (options.control_mode == "playback" && options.playback_file.empty()) {
        throw std::invalid_argument("--control-mode playback requires --playback-file");
    }
    return options;
}

bool motion_authorized(const Options& options) {
    const char* env = std::getenv("JAKA_ENABLE_MOTION");
    const bool env_enabled = env != nullptr && std::string(env) == "1";
    if (options.arm_motion && !env_enabled) {
        throw std::runtime_error("--arm-motion requires JAKA_ENABLE_MOTION=1");
    }
    return options.arm_motion && env_enabled;
}

}  // namespace

int main(int argc, char** argv) {
    std::cout.setf(std::ios::unitbuf);
    std::cerr.setf(std::ios::unitbuf);
    windows_jaka::StopController stop;
    g_stop = &stop;
    SetConsoleCtrlHandler(console_handler, TRUE);
    JAKAZuRobot robot;
    bool logged_in = false;
    bool servo_enabled = false;
    std::mutex sdk_mutex;
    std::thread state_thread;
    std::thread control_thread;
    std::atomic<bool> control_stopping{false};
    std::atomic<bool> state_ok{false};
    std::atomic<bool> state_failed{false};
    std::atomic<bool> control_pipe_failed{false};
    std::atomic<int> cached_status_ret{-1};
    std::atomic<int> cached_drag_ret{-1};
    std::atomic<std::uint8_t> cached_powered{0};
    std::atomic<std::uint8_t> cached_enabled{0};
    std::atomic<std::uint8_t> cached_dragging{0};
    windows_jaka::ControlMailbox control_mailbox;
    std::wstring control_pipe_name{L"\\\\.\\pipe\\jaka_operator_teleop"};

    try {
        const Options options = parse_options(argc, argv);
        control_pipe_name = options.control_pipe;
        const bool real_motion = motion_authorized(options);
        const bool joint_enabled = real_motion &&
            (options.control_mode == "joint" || options.control_mode == "record");
        const bool record_enabled = options.control_mode == "record";
        const bool playback_enabled = real_motion && options.control_mode == "playback";

        windows_jaka::WinsockRuntime winsock;
        windows_jaka::UdpSocket udp;
        std::vector<windows_jaka::UdpEndpoint> destinations;
        if (options.peer_ports.empty()) {
            destinations.push_back({"127.0.0.1", options.port});
        } else {
            for (std::uint16_t peer_port : options.peer_ports) {
                destinations.push_back({"127.0.0.1", peer_port});
            }
        }
        udp.open_sender_multi(destinations);
        std::cout << "operator_udp_destinations=";
        for (std::size_t i = 0; i < destinations.size(); ++i) {
            if (i != 0) std::cout << ',';
            std::cout << destinations[i].port;
        }
        std::cout << "\n";

        const int login_ret = robot.login_in(options.operator_ip.c_str(), false);
        logged_in = login_ret == 0;
        std::cout << "WINDOWS_JAKA_TIMING_V2\n";
        if (GetModuleHandleA("jakaAPI.dll") == nullptr) {
            throw std::runtime_error("jakaAPI.dll is not loaded");
        }
        std::cout << "jakaAPI.dll load=ok\n";
        std::cout << "operator login_ret=" << login_ret
                  << " ip=" << options.operator_ip << "\n";
        if (login_ret != 0) return 1;

        RobotStatus_simple initial_status{};
        BOOL initial_dragging = FALSE;
        JointValue initial_joints{};
        {
            std::lock_guard<std::mutex> lock(sdk_mutex);
            const int status_ret = robot.get_robot_status_simple(&initial_status);
            const int drag_ret = robot.is_in_drag_mode(&initial_dragging);
            const int joint_ret = robot.get_actual_joint_position(&initial_joints);
            if (status_ret != 0 || drag_ret != 0 || joint_ret != 0) {
                std::cerr << "initial operator state read failed status_ret=" << status_ret
                          << " drag_ret=" << drag_ret
                          << " joint_ret=" << joint_ret << "\n";
                robot.login_out();
                logged_in = false;
                return 2;
            }
        }
        if (joint_enabled && initial_dragging) {
            throw std::runtime_error("operator must exit drag mode before joint control");
        }
        if ((joint_enabled || playback_enabled) && initial_dragging) {
            throw std::runtime_error("operator must exit drag mode before joint or trajectory control");
        }

        std::cout << (real_motion ? "REAL ROBOT MOTION ENABLED\n"
                                  : "DRY-RUN ONLY: operator servo_j is disabled\n");
        std::cout << "operator_control_mode=" << options.control_mode << "\n";
        std::cout << "operator_servo_filter=" << options.filter
                  << " lpf_cutoff=" << options.lpf_cutoff
                  << " configure_ret=" << (real_motion ? "PENDING" : "SKIPPED_DRY_RUN") << "\n";
        std::cout << "operator_software_limits max_velocity_rad_s=" << options.max_velocity
                  << " max_acceleration_rad_s2=" << options.max_acceleration
                  << " max_step_rad=[0.008,0.008,0.008,0.006,0.006,0.006]\n";
        if (!options.record_file.empty()) std::cout << "operator_record_file=" << options.record_file << "\n";
        if (!options.playback_file.empty()) std::cout << "operator_playback_file=" << options.playback_file << " speed=" << options.playback_speed << "\n";

        state_ok.store(false);
        cached_status_ret.store(0);
        cached_drag_ret.store(0);
        cached_powered.store(initial_status.powered_on ? 1 : 0);
        cached_enabled.store(initial_status.enabled ? 1 : 0);
        cached_dragging.store(initial_dragging ? 1 : 0);
        state_ok.store(true);

        state_thread = std::thread([&] {
            try {
                while (!stop.stopping()) {
                    RobotStatus_simple status{};
                    BOOL dragging = FALSE;
                    int status_ret = -1;
                    int drag_ret = -1;
                    {
                        std::lock_guard<std::mutex> lock(sdk_mutex);
                        status_ret = robot.get_robot_status_simple(&status);
                        drag_ret = robot.is_in_drag_mode(&dragging);
                    }
                    cached_status_ret.store(status_ret);
                    cached_drag_ret.store(drag_ret);
                    if (status_ret == 0) {
                        cached_powered.store(status.powered_on ? 1 : 0);
                        cached_enabled.store(status.enabled ? 1 : 0);
                    }
                    if (drag_ret == 0) cached_dragging.store(dragging ? 1 : 0);
                    state_ok.store(status_ret == 0 && drag_ret == 0);
                    stop.wait_for(std::chrono::milliseconds(40));
                }
            } catch (...) {
                state_ok.store(false);
                state_failed.store(true);
                stop.request_stop();
            }
        });

        control_thread = std::thread([&] {
            try {
                windows_jaka::ControlPipeServer server(options.control_pipe, control_mailbox);
                server.run(control_stopping);
            } catch (...) {
                control_pipe_failed.store(true);
                stop.request_stop();
            }
        });

        windows_jaka::JointArray initial_position{};
        for (int i = 0; i < 6; ++i) initial_position[i] = initial_joints.jVal[i];
        windows_jaka::JointSafetyLimits safety_limits;
        safety_limits.max_velocity.fill(options.max_velocity);
        safety_limits.max_acceleration.fill(options.max_acceleration);
        windows_jaka::JointLimiter limiter(safety_limits);
        windows_jaka::OnePoleLowPass low_pass(0.85);
        // Initialize only when a motion command is actually requested.
        std::cout << std::fixed << std::setprecision(10)
                  << "operator_zero_raw_rad=[";
        for (int i = 0; i < 6; ++i) {
            if (i != 0) std::cout << ", ";
            std::cout << initial_position[i];
        }
        std::cout << "] operator_zero_deg_if_rad=[";
        for (int i = 0; i < 6; ++i) {
            if (i != 0) std::cout << ", ";
            std::cout << initial_position[i] * 180.0 / 3.14159265358979323846;
        }
        std::cout << "]\n";

        windows_jaka::JointArray last_target = initial_position;
        windows_jaka::JointArray manual_jog_delta{};
        bool manual_jog_active = false;
        std::uint64_t last_manual_jog_ns = 0;
        std::string stop_reason;
        bool duration_elapsed = false;

        std::vector<windows_jaka::TrajectoryPoint> playback_points;
        windows_jaka::JointArray playback_offset{};
        double playback_duration_sec = 0.0;
        std::uint64_t playback_start_ns = 0;
        windows_jaka::JointArray playback_target{};
        if (playback_enabled) {
            std::string trajectory_error;
            if (!windows_jaka::load_trajectory_file(options.playback_file, playback_points, trajectory_error)) {
                throw std::runtime_error("playback trajectory invalid: " + trajectory_error);
            }
            playback_duration_sec = windows_jaka::trajectory_duration_seconds(playback_points);
            for (int i = 0; i < 6; ++i) {
                playback_offset[i] = initial_position[i] - playback_points.front().joints[i];
            }
            std::cout << "operator_playback_samples=" << playback_points.size()
                      << " duration_s=" << playback_duration_sec
                      << " mode=relative\n";
        }

        auto configure_servo_filter = [&]() -> int {
            if (options.filter == "none") return robot.servo_move_use_none_filter();
            if (options.filter == "lpf") return robot.servo_move_use_joint_LPF(options.lpf_cutoff);
            return robot.servo_move_use_joint_NLF(45.0, 600.0, 6000.0);
        };

        constexpr std::uint64_t period_ns = 8'000'000ULL;
        const std::uint64_t start_ns = windows_jaka::monotonic_ns();
        std::uint64_t next_ns = start_ns;
        std::uint64_t sequence = 0;
        std::uint64_t sent = 0;
        std::uint64_t read_errors = 0;
        std::uint64_t send_errors = 0;
        std::ofstream record;
        if (record_enabled) {
            record.open(options.record_file, std::ios::out | std::ios::trunc);
            if (!record.is_open()) throw std::runtime_error("cannot open record file");
            windows_jaka::write_trajectory_header(record);
        }

        while (!stop.stopping()) {
            const std::uint64_t tick_ns = windows_jaka::monotonic_ns();
            if (options.duration_sec > 0.0 &&
                tick_ns - start_ns >= static_cast<std::uint64_t>(options.duration_sec * 1e9)) {
                duration_elapsed = true;
                break;
            }

            JointValue joints{};
            int joint_ret = -1;
            {
                std::lock_guard<std::mutex> lock(sdk_mutex);
                joint_ret = robot.get_actual_joint_position(&joints);
            }

            windows_jaka::ControlCommand control_command{};
            while (control_mailbox.take(control_command)) {
                if (control_command.type == windows_jaka::ControlCommand::Type::Jog) {
                    if (!joint_enabled) {
                        std::cout << "operator JOG ignored: control_mode=" << options.control_mode << "\n";
                        continue;
                    }
                    if (cached_dragging.load()) {
                        std::cout << "operator JOG ignored: drag mode is active\n";
                        continue;
                    }
                    manual_jog_delta[control_command.axis] = control_command.delta_rad;
                    manual_jog_active = true;
                    last_manual_jog_ns = tick_ns;
                    if (!servo_enabled) {
                        if (joint_ret != 0) {
                            stop_reason = "OPERATOR_INITIAL_JOINT_READ_FAILED";
                            stop.request_stop();
                            break;
                        }
                        windows_jaka::JointArray current{};
                        for (int i = 0; i < 6; ++i) current[i] = joints.jVal[i];
                        if (!limiter.within_limits(current)) {
                            stop_reason = "JOINT_LIMIT_VIOLATION";
                            stop.request_stop();
                            break;
                        }
                        limiter.reset(current);
                        low_pass.reset(current);
                        last_target = current;
                        int configure_ret = 0;
                        int enable_ret = 0;
                        {
                            std::lock_guard<std::mutex> lock(sdk_mutex);
                            configure_ret = configure_servo_filter();
                            if (configure_ret == 0) enable_ret = robot.servo_move_enable(TRUE, false);
                        }
                        std::cout << "operator_servo_filter=" << options.filter
                                  << " lpf_cutoff=" << options.lpf_cutoff
                                  << " configure_ret=" << configure_ret << "\n";
                        if (configure_ret != 0) {
                            stop_reason = "SERVO_FILTER_CONFIG_FAILED";
                            stop.request_stop();
                            break;
                        }
                        if (enable_ret != 0) {
                            stop_reason = "SERVO_ENABLE_FAILED";
                            stop.request_stop();
                            break;
                        }
                        servo_enabled = true;
                    }
                } else if (control_command.type == windows_jaka::ControlCommand::Type::JogStop) {
                    manual_jog_delta[control_command.axis] = 0.0;
                    bool any_jog = false;
                    for (double value : manual_jog_delta) {
                        if (std::abs(value) > 1e-12) any_jog = true;
                    }
                    manual_jog_active = any_jog;
                    last_manual_jog_ns = tick_ns;
                } else if (control_command.type == windows_jaka::ControlCommand::Type::Stop) {
                    stop.request_stop();
                    break;
                } else if (control_command.type == windows_jaka::ControlCommand::Type::SafetyPose) {
                    if (!joint_enabled) {
                        std::cout << "operator SAFEPOSE ignored: control_mode=" << options.control_mode << "\n";
                        continue;
                    }
                    if (cached_dragging.load()) {
                        std::cout << "operator SAFEPOSE ignored: drag mode is active\n";
                        continue;
                    }
                    windows_jaka::JointArray requested{};
                    for (int i = 0; i < 6; ++i) requested[i] = control_command.pose[i];
                    if (!limiter.within_limits(requested)) {
                        std::cout << "operator SAFEPOSE rejected: joint limit violation\n";
                        continue;
                    }
                    JointValue pose{};
                    for (int i = 0; i < 6; ++i) pose.jVal[i] = requested[i];
                    int pose_ret = -1;
                    {
                        std::lock_guard<std::mutex> lock(sdk_mutex);
                        if (servo_enabled) {
                            robot.motion_abort();
                            robot.servo_move_enable(FALSE, false);
                            servo_enabled = false;
                        }
                        pose_ret = robot.joint_move(&pose, ABS, TRUE, 0.2, 0.5);
                    }
                    std::cout << "operator joint_move_ret=" << pose_ret << "\n";
                    if (pose_ret != 0) stop_reason = "SAFETY_POSE_FAILED";
                    stop.request_stop();
                    break;
                }
            }

            if (stop.stopping()) break;

            if (joint_enabled && servo_enabled && cached_dragging.load()) {
                stop_reason = "OPERATOR_DRAG_MODE_DURING_JOINT";
                stop.request_stop();
                break;
            }

            if (manual_jog_active && tick_ns > last_manual_jog_ns &&
                tick_ns - last_manual_jog_ns > 250'000'000ULL) {
                manual_jog_delta.fill(0.0);
                manual_jog_active = false;
            }

            if (manual_jog_active) {
                if (!cached_powered.load() || !cached_enabled.load() ||
                    cached_status_ret.load() != 0 || cached_drag_ret.load() != 0) {
                    stop_reason = "OPERATOR_NOT_READY";
                    stop.request_stop();
                    break;
                }
                windows_jaka::JointArray desired = last_target;
                for (int i = 0; i < 6; ++i) desired[i] += manual_jog_delta[i];
                if (!limiter.within_limits(desired)) {
                    stop_reason = "JOINT_LIMIT_VIOLATION";
                    stop.request_stop();
                    break;
                }
                const auto filtered = low_pass.filter(desired);
                const auto target = limiter.step(filtered, 0.008);
                last_target = target;
                JointValue command{};
                for (int i = 0; i < 6; ++i) command.jVal[i] = last_target[i];
                int servo_ret = -1;
                {
                    std::lock_guard<std::mutex> lock(sdk_mutex);
                    servo_ret = robot.servo_j(&command, ABS, 1);
                }
                if (servo_ret != 0) {
                    stop_reason = "SERVO_J_FAILED";
                    stop.request_stop();
                    break;
                }
            }

            if (playback_enabled) {
                if (joint_ret != 0) {
                    stop_reason = "PLAYBACK_JOINT_READ_FAILED";
                    stop.request_stop();
                    break;
                }
                if (!cached_powered.load() || !cached_enabled.load() ||
                    cached_status_ret.load() != 0 || cached_drag_ret.load() != 0) {
                    stop_reason = "PLAYBACK_OPERATOR_NOT_READY";
                    stop.request_stop();
                    break;
                }

                if (!servo_enabled) {
                    windows_jaka::JointArray current{};
                    for (int i = 0; i < 6; ++i) current[i] = joints.jVal[i];
                    if (!limiter.within_limits(current)) {
                        stop_reason = "PLAYBACK_START_LIMIT_VIOLATION";
                        stop.request_stop();
                        break;
                    }
                    limiter.reset(current);
                    low_pass.reset(current);
                    last_target = current;
                    int configure_ret = 0;
                    int enable_ret = 0;
                    {
                        std::lock_guard<std::mutex> lock(sdk_mutex);
                        configure_ret = configure_servo_filter();
                        if (configure_ret == 0) enable_ret = robot.servo_move_enable(TRUE, false);
                    }
                    if (configure_ret != 0) {
                        stop_reason = "SERVO_FILTER_CONFIG_FAILED";
                        stop.request_stop();
                        break;
                    }
                    if (enable_ret != 0) {
                        stop_reason = "SERVO_ENABLE_FAILED";
                        stop.request_stop();
                        break;
                    }
                    servo_enabled = true;
                    playback_start_ns = tick_ns;
                }

                const double playback_elapsed_s =
                    static_cast<double>(tick_ns - playback_start_ns) / 1e9 * options.playback_speed;
                const auto sampled = windows_jaka::sample_trajectory(playback_points, playback_elapsed_s);
                for (int i = 0; i < 6; ++i) playback_target[i] = sampled[i] + playback_offset[i];
                if (!limiter.within_limits(playback_target)) {
                    stop_reason = "PLAYBACK_JOINT_LIMIT_VIOLATION";
                    stop.request_stop();
                    break;
                }

                const auto filtered = low_pass.filter(playback_target);
                last_target = limiter.step(filtered, 0.008);
                JointValue command{};
                for (int i = 0; i < 6; ++i) command.jVal[i] = last_target[i];
                int servo_ret = -1;
                {
                    std::lock_guard<std::mutex> lock(sdk_mutex);
                    servo_ret = robot.servo_j(&command, ABS, 1);
                }
                if (servo_ret != 0) {
                    stop_reason = "SERVO_J_FAILED";
                    stop.request_stop();
                    break;
                }

                double target_error = 0.0;
                for (int i = 0; i < 6; ++i) {
                    target_error = std::max(target_error, std::abs(last_target[i] - playback_target[i]));
                }
                if (playback_elapsed_s >= playback_duration_sec && target_error < 1e-4) {
                    duration_elapsed = true;
                    break;
                }
            }

            windows_jaka::JointSamplePacket packet = windows_jaka::make_empty_packet();
            packet.sequence = sequence++;
            packet.monotonic_ns = windows_jaka::monotonic_ns();
            packet.sdk_code = joint_ret;
            packet.operator_powered = cached_powered.load();
            packet.operator_enabled = cached_enabled.load();
            packet.operator_dragging = cached_dragging.load();
            packet.operator_valid =
                joint_ret == 0 && cached_status_ret.load() == 0 &&
                cached_drag_ret.load() == 0 && state_ok.load() ? 1 : 0;
            if (joint_ret == 0) {
                for (int i = 0; i < 6; ++i) packet.position[i] = joints.jVal[i];
                if (!windows_jaka::finite_position(packet)) packet.operator_valid = 0;
            } else {
                ++read_errors;
            }
            if (udp.send_packet(packet)) ++sent;
            else ++send_errors;
            if (record.is_open() && joint_ret == 0) {
                windows_jaka::JointArray recorded{};
                for (int i = 0; i < 6; ++i) recorded[i] = joints.jVal[i];
                const std::uint64_t time_ms = (tick_ns - start_ns) / 1'000'000ULL;
                windows_jaka::write_trajectory_point(record, time_ms, recorded);
            }

            if (packet.sequence % 125 == 0) {
                const double elapsed_s = static_cast<double>(packet.monotonic_ns - start_ns) / 1e9;
                std::cout << std::fixed << std::setprecision(2)
                          << "operator status seq=" << packet.sequence
                          << " rate_hz=" << (elapsed_s > 0.0 ? packet.sequence / elapsed_s : 0.0)
                          << " powered=" << static_cast<int>(packet.operator_powered)
                          << " enabled=" << static_cast<int>(packet.operator_enabled)
                          << " dragging=" << static_cast<int>(packet.operator_dragging)
                          << " valid=" << static_cast<int>(packet.operator_valid)
                          << " joint_jog=" << (manual_jog_active ? 1 : 0)
                          << " record=" << (record_enabled ? 1 : 0)
                          << " playback=" << (playback_enabled ? 1 : 0)
                          << "\n";
            }

            next_ns += period_ns;
            const std::uint64_t after_work = windows_jaka::monotonic_ns();
            if (next_ns > after_work) {
                std::this_thread::sleep_for(std::chrono::nanoseconds(next_ns - after_work));
            } else {
                next_ns = after_work;
            }
        }

        stop.request_stop();
        if (state_thread.joinable()) state_thread.join();
        if (servo_enabled) {
            robot.motion_abort();
            robot.servo_move_enable(FALSE, false);
            servo_enabled = false;
        }
        control_stopping.store(true);
        windows_jaka::send_control_command(options.control_pipe, "STOP");
        if (control_thread.joinable()) control_thread.join();
        const int logout_ret = robot.login_out();
        logged_in = false;

        if (stop_reason.empty() && state_failed.load()) stop_reason = "STATE_THREAD_EXCEPTION";
        if (stop_reason.empty() && control_pipe_failed.load()) stop_reason = "CONTROL_PIPE_EXCEPTION";
        if (stop_reason.empty() && !duration_elapsed) stop_reason = "STOP_REQUESTED";
        if (record.is_open()) record.flush();
        std::cout << "operator summary samples=" << sequence
                  << " playback_samples=" << playback_points.size()
                  << " sent=" << sent
                  << " read_errors=" << read_errors
                  << " send_errors=" << send_errors
                  << " logout_ret=" << logout_ret << "\n";
        std::cout << "operator stop_reason="
                  << (stop_reason.empty() ? "DURATION" : stop_reason) << "\n";
        return logout_ret == 0 && sent > 0 && send_errors == 0 ? 0 : 3;
    } catch (const std::exception& error) {
        stop.request_stop();
        if (state_thread.joinable()) state_thread.join();
        if (servo_enabled) {
            robot.motion_abort();
            robot.servo_move_enable(FALSE, false);
            servo_enabled = false;
        }
        control_stopping.store(true);
        windows_jaka::send_control_command(control_pipe_name, "STOP");
        if (control_thread.joinable()) control_thread.join();
        if (logged_in) robot.login_out();
        std::cerr << "windows_operator fatal: " << error.what() << "\n";
        return 10;
    }
}
