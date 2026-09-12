#include <windows.h>

#include "JAKAZuRobot.h"
#include "control_pipe.hpp"
#include "joint_mapping.hpp"
#include "joint_sample_packet.hpp"
#include "runtime_control.hpp"
#include "runtime_status.hpp"
#include "trajectory.hpp"
#include "udp_transport.hpp"
#include "waitable_timer.hpp"
#include "windows_clock.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
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
    std::string robot_id{"follower"};
    std::string follower_ip{"192.168.0.102"};
    std::uint16_t port{30001};
    double duration_sec{0.0};
    std::string record_file;
    std::string playback_file;
    std::string status_file;
    double playback_speed{1.0};
    bool arm_motion{false};
    bool offline{false};
    std::string filter{"none"};
    double lpf_cutoff{2.5};
    double max_velocity{1.0};
    double max_acceleration{8.0};
    std::wstring control_pipe{L"\\\\.\\pipe\\jaka_dual_teleop"};
    std::string control_mode{"teleop"};
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
        if (arg == "--robot-id") options.robot_id = next("--robot-id");
        else if (arg == "--follower-ip") options.follower_ip = next("--follower-ip");
        else if (arg == "--port") options.port = parse_port(next("--port"));
        else if (arg == "--duration-sec") options.duration_sec = std::stod(next("--duration-sec"));
        else if (arg == "--record-file") options.record_file = next("--record-file");
        else if (arg == "--status-file") options.status_file = next("--status-file");
        else if (arg == "--playback-file") options.playback_file = next("--playback-file");
        else if (arg == "--playback-speed") options.playback_speed = std::stod(next("--playback-speed"));
        else if (arg == "--arm-motion") options.arm_motion = true;
        else if (arg == "--dry-run") options.arm_motion = false;
        else if (arg == "--offline") options.offline = true;
        else if (arg == "--filter") options.filter = next("--filter");
        else if (arg == "--lpf-cutoff") options.lpf_cutoff = std::stod(next("--lpf-cutoff"));
        else if (arg == "--max-velocity") options.max_velocity = std::stod(next("--max-velocity"));
        else if (arg == "--max-acceleration") options.max_acceleration = std::stod(next("--max-acceleration"));
        else if (arg == "--control-pipe") {
            const std::string value = next("--control-pipe");
            options.control_pipe.assign(value.begin(), value.end());
        }
        else if (arg == "--control-mode") options.control_mode = next("--control-mode");
        else if (arg == "--help" || arg == "-h") {
            std::cout << "windows_follower.exe [--follower-ip IP] [--port N] [--duration-sec S]"
                         " [--robot-id ID] [--status-file PATH]"
                         " [--record-file PATH] [--playback-file PATH] [--playback-speed 1.0]"
                         " [--dry-run|--arm-motion] [--offline]"
                         " [--filter none|lpf|nlf] [--lpf-cutoff 2.5]"
                         " [--max-velocity 1.0] [--max-acceleration 8.0]"
                         " [--control-pipe \\\\.\\pipe\\jaka_dual_teleop]"
                         " [--control-mode idle|teleop|joint|record|playback]\n";
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
    if (options.offline && options.arm_motion) {
        throw std::runtime_error("--offline cannot be combined with --arm-motion");
    }
    return options.arm_motion && env_enabled;
}

struct LatestPacketMailbox {
    void publish(const windows_jaka::JointSamplePacket& packet, std::uint64_t received_ns) {
        std::lock_guard<std::mutex> lock(mutex);
        if (has_packet && packet.sequence <= latest.sequence) {
            ++invalid_sequence_count;
            return;
        }
        if (has_packet) {
            if (packet.sequence > latest.sequence + 1) dropped_packets += packet.sequence - latest.sequence - 1;
            const auto interval = received_ns - latest_received_ns;
            receive_interval_sum_ns += interval;
            if (interval > max_receive_interval_ns) max_receive_interval_ns = interval;
        }
        latest = packet;
        latest_received_ns = received_ns;
        has_packet = true;
        ++receive_samples;
    }

    bool snapshot(windows_jaka::JointSamplePacket& packet, std::uint64_t& received_ns,
                  std::uint64_t& dropped, std::uint64_t& invalid_sequences,
                  std::uint64_t& interval_sum, std::uint64_t& interval_max,
                  std::uint64_t& samples) const {
        std::lock_guard<std::mutex> lock(mutex);
        if (!has_packet) return false;
        packet = latest;
        received_ns = latest_received_ns;
        dropped = dropped_packets;
        invalid_sequences = invalid_sequence_count;
        interval_sum = receive_interval_sum_ns;
        interval_max = max_receive_interval_ns;
        samples = receive_samples;
        return true;
    }

    mutable std::mutex mutex;
    windows_jaka::JointSamplePacket latest{};
    std::uint64_t latest_received_ns{0};
    std::uint64_t dropped_packets{0};
    std::uint64_t invalid_sequence_count{0};
    std::uint64_t receive_interval_sum_ns{0};
    std::uint64_t max_receive_interval_ns{0};
    std::uint64_t receive_samples{0};
    bool has_packet{false};
};

struct FaultState {
    void set(const std::string& value) {
        std::lock_guard<std::mutex> lock(mutex);
        if (reason.empty()) reason = value;
        fault.store(true);
    }
    std::string get() const {
        std::lock_guard<std::mutex> lock(mutex);
        return reason;
    }
    std::atomic<bool> fault{false};
    mutable std::mutex mutex;
    std::string reason;
};

}  // namespace

int main(int argc, char** argv) {
    std::cout.setf(std::ios::unitbuf);
    std::cerr.setf(std::ios::unitbuf);
    windows_jaka::StopController stop;
    g_stop = &stop;
    SetConsoleCtrlHandler(console_handler, TRUE);
    std::thread receiver_thread;
    std::thread control_thread;
    std::atomic<bool> control_stopping{false};
    std::thread sdk_thread;
    JAKAZuRobot robot;
    bool logged_in = false;
    bool servo_enabled = false;
    LatestPacketMailbox mailbox;
    windows_jaka::ControlMailbox control_mailbox;
    FaultState fault;
    std::atomic<int> sdk_result{0};
    std::wstring control_pipe_name{L"\\\\.\\pipe\\jaka_dual_teleop"};

    try {
        const Options options = parse_options(argc, argv);
        control_pipe_name = options.control_pipe;
        const bool real_motion = motion_authorized(options);
        const bool teleop_enabled = real_motion && options.control_mode == "teleop";
        const bool joint_enabled = real_motion &&
            (options.control_mode == "joint" || options.control_mode == "record");
        const bool record_enabled = options.control_mode == "record";
        const bool playback_enabled = real_motion && options.control_mode == "playback";
        std::cout << "WINDOWS_JAKA_TIMING_V2\n";
        if (real_motion) std::cout << "REAL ROBOT MOTION ENABLED\n";
        else std::cout << "DRY-RUN ONLY: servo_j is disabled\n";
        std::cout << "control_mode=" << options.control_mode << "\n";
        std::cout << "servo_filter=" << options.filter
                  << " lpf_cutoff=" << options.lpf_cutoff
                  << " configure_ret=" << (real_motion ? "PENDING" : "SKIPPED_DRY_RUN") << "\n";
        std::cout << "software_limits max_velocity_rad_s=" << options.max_velocity
                  << " max_acceleration_rad_s2=" << options.max_acceleration
                  << " max_step_rad=[0.008,0.008,0.008,0.006,0.006,0.006]\n";
        if (!options.record_file.empty()) std::cout << "record_file=" << options.record_file << "\n";
        if (!options.playback_file.empty()) std::cout << "playback_file=" << options.playback_file << " speed=" << options.playback_speed << "\n";

        windows_jaka::WinsockRuntime winsock;
        windows_jaka::UdpSocket udp;
        udp.open_receiver(options.port);

        receiver_thread = std::thread([&] {
            try {
                while (!stop.stopping()) {
                    windows_jaka::JointSamplePacket packet{};
                    const auto status = udp.receive_packet_status(packet, 10);
                    if (status == windows_jaka::UdpSocket::ReceiveStatus::Timeout) continue;
                    if (status == windows_jaka::UdpSocket::ReceiveStatus::InvalidDatagram) {
                        fault.set("INVALID_UDP_DATAGRAM");
                        stop.request_stop();
                        break;
                    }
                    if (!windows_jaka::valid_packet(packet)) {
                        fault.set("INVALID_OPERATOR_PACKET");
                        stop.request_stop();
                        break;
                    }
                    mailbox.publish(packet, windows_jaka::monotonic_ns());
                }
            } catch (...) {
                fault.set("UDP_RECEIVER_EXCEPTION");
                stop.request_stop();
            }
        });

        control_thread = std::thread([&] {
            try {
                windows_jaka::ControlPipeServer server(options.control_pipe, control_mailbox);
                server.run(control_stopping);
            } catch (...) {
                fault.set("CONTROL_PIPE_EXCEPTION");
                stop.request_stop();
            }
        });

        sdk_thread = std::thread([&] {
            int abort_ret = 0;
            int disable_ret = 0;
            int logout_ret = 0;
            std::uint64_t servo_calls = 0;
            std::uint64_t servo_interval_sum_ns = 0;
            std::uint64_t servo_interval_max_ns = 0;
            std::uint64_t max_servo_call_ns = 0;
            std::uint64_t packet_age_sum_ns = 0;
            std::uint64_t packet_age_max_ns = 0;
            std::uint64_t packet_age_samples = 0;
            int configure_ret = 0;
            std::uint64_t previous_servo_ns = 0;
            std::uint64_t hold_ticks = 0;
            std::uint64_t watchdog_ticks = 0;
            windows_jaka::JointArray max_target_delta{};
            windows_jaka::JointArray max_command_error{};
            windows_jaka::JointArray follower_zero{};
            windows_jaka::JointArray operator_zero{};
            windows_jaka::JointArray last_target{};
            windows_jaka::JointArray latest_mapped{};
            windows_jaka::JointArray manual_jog_delta{};
            bool have_operator_zero = false;
            bool have_latest_mapped = false;
            bool latest_dragging = false;
            bool manual_jog_active = false;
            std::uint64_t latest_operator_timestamp_ns = 0;
            std::uint64_t last_sequence = 0;
            std::uint64_t last_processed_received_ns = 0;
            std::uint64_t last_manual_jog_ns = 0;
            std::uint64_t status_tick = 0;
            int status_login_ret = -1;
            int status_servo_error = 0;
            std::uint64_t latest_dropped_packets = 0;
            bool status_robot_powered = false;
            bool status_robot_enabled = false;
            bool status_robot_dragging = false;
            std::string status_write_error_reported;
            const std::uint64_t start_ns = windows_jaka::monotonic_ns();
            std::vector<windows_jaka::TrajectoryPoint> playback_points;
            windows_jaka::JointArray playback_offset{};
            windows_jaka::JointArray playback_target{};
            double playback_duration_sec = 0.0;
            std::uint64_t playback_start_ns = 0;
            std::ofstream record;
            try {
                if (!options.offline) {
                    const int login_ret = robot.login_in(options.follower_ip.c_str(), false);
                    logged_in = login_ret == 0;
                    status_login_ret = login_ret;
                    if (GetModuleHandleA("jakaAPI.dll") == nullptr) throw std::runtime_error("jakaAPI.dll is not loaded");
                    std::cout << "jakaAPI.dll load=ok\n";
                    std::cout << "follower login_ret=" << login_ret << " ip=" << options.follower_ip << "\n";
                    if (login_ret != 0) throw std::runtime_error("follower login failed");

                    RobotStatus_simple status{};
                    const int status_ret = robot.get_robot_status_simple(&status);
                    if (status_ret != 0 || !status.powered_on || !status.enabled) throw std::runtime_error("follower must already be powered and enabled");
                    status_robot_powered = status.powered_on != 0;
                    status_robot_enabled = status.enabled != 0;
                    JointValue initial{};
                    const int initial_ret = robot.get_actual_joint_position(&initial);
                    if (initial_ret != 0) throw std::runtime_error("follower initial joint read failed");
                    for (int i = 0; i < 6; ++i) follower_zero[i] = initial.jVal[i];
                }

                if (playback_enabled) {
                    std::string trajectory_error;
                    if (!windows_jaka::load_trajectory_file(options.playback_file, playback_points, trajectory_error)) {
                        throw std::runtime_error("playback trajectory invalid: " + trajectory_error);
                    }
                    playback_duration_sec = windows_jaka::trajectory_duration_seconds(playback_points);
                    for (int i = 0; i < 6; ++i) {
                        playback_offset[i] = follower_zero[i] - playback_points.front().joints[i];
                    }
                    std::cout << "follower_playback_samples=" << playback_points.size()
                              << " duration_s=" << playback_duration_sec
                              << " mode=relative\n";
                }
                if (record_enabled) {
                    record.open(options.record_file, std::ios::out | std::ios::trunc);
                    if (!record.is_open()) throw std::runtime_error("cannot open record file");
                    windows_jaka::write_trajectory_header(record);
                }

                windows_jaka::JointSafetyLimits safety_limits;
                safety_limits.max_velocity.fill(options.max_velocity);
                safety_limits.max_acceleration.fill(options.max_acceleration);
                windows_jaka::JointLimiter limiter(safety_limits);
                windows_jaka::OnePoleLowPass low_pass(0.85);
                // Initialize only when a motion command is actually requested.
                last_target = follower_zero;
                std::cout << std::fixed << std::setprecision(10)
                          << "follower_zero_raw_rad=[";
                for (int i = 0; i < 6; ++i) {
                    if (i != 0) std::cout << ", ";
                    std::cout << follower_zero[i];
                }
                std::cout << "] follower_zero_deg_if_rad=[";
                for (int i = 0; i < 6; ++i) {
                    if (i != 0) std::cout << ", ";
                    std::cout << follower_zero[i] * 180.0 / 3.14159265358979323846;
                }
                std::cout << "]\n";
                windows_jaka::PeriodicWaitableTimer timer(8);
                while (!stop.stopping()) {
                    timer.wait();
                    const std::uint64_t tick_ns = windows_jaka::monotonic_ns();
                    if (options.duration_sec > 0.0 && tick_ns - start_ns >= static_cast<std::uint64_t>(options.duration_sec * 1e9)) break;
                    if (fault.fault.load()) break;
                    JointValue record_joints{};
                    int record_ret = -1;
                    if (record_enabled) {
                        record_ret = robot.get_actual_joint_position(&record_joints);
                    }

                    windows_jaka::ControlCommand control_command{};
                    while (control_mailbox.take(control_command)) {
                        if (control_command.type == windows_jaka::ControlCommand::Type::Jog) {
                            if (!joint_enabled) continue;
                            manual_jog_delta[control_command.axis] = control_command.delta_rad;
                            manual_jog_active = true;
                            // Use the tick timestamp, never a fresh monotonic_ns()
                            // call: age_ns below is computed as
                            // tick_ns - last_processed_received_ns, and a
                            // timestamp taken after tick_ns wraps the unsigned
                            // subtraction to ~2^64, which tripped a bogus
                            // WATCHDOG_100MS and killed the session on the very
                            // first jog press.
                            last_processed_received_ns = tick_ns;
                            latest_operator_timestamp_ns = tick_ns;
                            last_manual_jog_ns = tick_ns;
                            if (joint_enabled && !servo_enabled) {
                                latest_dragging = true;
                                latest_mapped = last_target;
                                have_latest_mapped = true;
                                limiter.reset(last_target);
                                low_pass.reset(last_target);
                                if (options.filter == "none") {
                                    configure_ret = robot.servo_move_use_none_filter();
                                } else if (options.filter == "lpf") {
                                    configure_ret = robot.servo_move_use_joint_LPF(options.lpf_cutoff);
                                } else {
                                    configure_ret = robot.servo_move_use_joint_NLF(45.0, 600.0, 6000.0);
                                }
                                if (configure_ret != 0) {
                                    fault.set("SERVO_FILTER_CONFIG_FAILED");
                                    break;
                                }
                                const int enable_ret = robot.servo_move_enable(TRUE, false);
                                if (enable_ret != 0) {
                                    fault.set("SERVO_ENABLE_FAILED");
                                    break;
                                }
                                servo_enabled = true;
                            }
                        } else if (control_command.type == windows_jaka::ControlCommand::Type::JogStop) {
                            manual_jog_delta[control_command.axis] = 0.0;
                            bool any_jog = false;
                            for (double value : manual_jog_delta) if (std::abs(value) > 1e-12) any_jog = true;
                            manual_jog_active = any_jog;
                            last_manual_jog_ns = tick_ns;
                        } else if (control_command.type == windows_jaka::ControlCommand::Type::Stop) {
                            stop.request_stop();
                        } else if (control_command.type == windows_jaka::ControlCommand::Type::SafetyPose) {
                            if (!real_motion) {
                                std::cout << "???? dry-run pose_received\n";
                            } else {
                                if (servo_enabled) {
                                    robot.motion_abort();
                                    robot.servo_move_enable(FALSE, false);
                                    servo_enabled = false;
                                }
                                std::cout << "???? joint_move ??\n";
                                JointValue pose{};
                                for (int i = 0; i < 6; ++i) pose.jVal[i] = control_command.pose[i];
                                const int pose_ret = robot.joint_move(&pose, ABS, TRUE, 0.2, 0.5);
                                std::cout << "???? joint_move_ret=" << pose_ret << "\n";
                                if (pose_ret != 0) fault.set("SAFETY_POSE_FAILED");
                                stop.request_stop();
                            }
                        }
                    }

                    windows_jaka::JointSamplePacket packet{};
                    std::uint64_t received_ns = 0, dropped = 0, invalid_sequences = 0, interval_sum = 0, interval_max = 0, samples = 0;
                    const bool have_packet = mailbox.snapshot(packet, received_ns, dropped, invalid_sequences, interval_sum, interval_max, samples);
                    if (invalid_sequences != 0) { fault.set("NON_MONOTONIC_SEQUENCE"); break; }

                    if (have_packet && received_ns != last_processed_received_ns) {
                        if (packet.sequence <= last_sequence && have_operator_zero) { fault.set("NON_MONOTONIC_SEQUENCE"); break; }
                        last_sequence = packet.sequence;
                        last_processed_received_ns = received_ns;
                        latest_operator_timestamp_ns = packet.monotonic_ns;
                        const bool dragging = packet.operator_dragging != 0;
                        latest_dragging = teleop_enabled && dragging;
                        if (teleop_enabled && (dragging || have_operator_zero) && (!packet.operator_powered || !packet.operator_enabled)) { fault.set("OPERATOR_NOT_READY"); break; }
                        if (teleop_enabled && dragging && !have_operator_zero) {
                            if (servo_enabled) follower_zero = last_target;
                            for (int i = 0; i < 6; ++i) operator_zero[i] = packet.position[i];
                            have_operator_zero = true;
                            limiter.reset(follower_zero);
                            low_pass.reset(follower_zero);
                            last_target = follower_zero;
                            std::cout << "operator zero captured sequence=" << packet.sequence << "\n";
                            if (teleop_enabled && !servo_enabled) {
                                if (options.filter == "none") {
                                    configure_ret = robot.servo_move_use_none_filter();
                                } else if (options.filter == "lpf") {
                                    configure_ret = robot.servo_move_use_joint_LPF(options.lpf_cutoff);
                                } else {
                                    configure_ret = robot.servo_move_use_joint_NLF(45.0, 600.0, 6000.0);
                                }
                                std::cout << "servo_filter=" << options.filter
                                          << " lpf_cutoff=" << options.lpf_cutoff
                                          << " configure_ret=" << configure_ret << "\n";
                                if (configure_ret != 0) {
                                    fault.set("SERVO_FILTER_CONFIG_FAILED");
                                    break;
                                }
                                const int enable_ret = robot.servo_move_enable(TRUE, false);
                                if (enable_ret != 0) { fault.set("SERVO_ENABLE_FAILED"); break; }
                                servo_enabled = true;
                            }
                        }
                        if (teleop_enabled && have_operator_zero && dragging && !manual_jog_active) {
                            windows_jaka::JointArray operator_position{};
                            for (int i = 0; i < 6; ++i) operator_position[i] = packet.position[i];
                            const auto mapped = windows_jaka::map_relative(operator_position, operator_zero, follower_zero);
                            if (!limiter.within_limits(mapped)) { fault.set("JOINT_LIMIT_VIOLATION"); break; }
                            latest_mapped = mapped;
                            have_latest_mapped = true;
                        }
                    }

                    // Manual jog is a hold-to-move command that the GUI refreshes
                    // every 8 ms. If a dropped JOG_STOP were the only thing that
                    // ended it, the target would creep forever, so expire it.
                    if (manual_jog_active && tick_ns > last_manual_jog_ns &&
                        tick_ns - last_manual_jog_ns > 250'000'000ULL) {
                        manual_jog_delta.fill(0.0);
                        manual_jog_active = false;
                    }

                    if (playback_enabled) {
                        windows_jaka::JointArray current = follower_zero;
                        if (record_ret == 0) {
                            for (int i = 0; i < 6; ++i) current[i] = record_joints.jVal[i];
                        }
                        if (!servo_enabled) {
                            if (!limiter.within_limits(current)) {
                                fault.set("PLAYBACK_START_LIMIT_VIOLATION");
                                break;
                            }
                            limiter.reset(current);
                            low_pass.reset(current);
                            last_target = current;
                            if (options.filter == "none") {
                                configure_ret = robot.servo_move_use_none_filter();
                            } else if (options.filter == "lpf") {
                                configure_ret = robot.servo_move_use_joint_LPF(options.lpf_cutoff);
                            } else {
                                configure_ret = robot.servo_move_use_joint_NLF(45.0, 600.0, 6000.0);
                            }
                            if (configure_ret != 0) {
                                fault.set("SERVO_FILTER_CONFIG_FAILED");
                                break;
                            }
                            const int enable_ret = robot.servo_move_enable(TRUE, false);
                            if (enable_ret != 0) {
                                fault.set("SERVO_ENABLE_FAILED");
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
                            fault.set("PLAYBACK_JOINT_LIMIT_VIOLATION");
                            break;
                        }
                        const auto filtered = low_pass.filter(playback_target);
                        const auto target = limiter.step(filtered, 0.008);
                        for (int i = 0; i < 6; ++i) {
                            const double delta = std::abs(target[i] - last_target[i]);
                            if (delta > max_target_delta[i]) max_target_delta[i] = delta;
                        }
                        last_target = target;
                        JointValue command{};
                        for (int i = 0; i < 6; ++i) command.jVal[i] = last_target[i];
                        const auto call_start_ns = windows_jaka::monotonic_ns();
                        const int servo_ret = robot.servo_j(&command, ABS, 1);
                        status_servo_error = servo_ret;
                        const auto call_end_ns = windows_jaka::monotonic_ns();
                        const auto call_duration_ns = call_end_ns - call_start_ns;
                        if (call_duration_ns > max_servo_call_ns) max_servo_call_ns = call_duration_ns;
                        if (previous_servo_ns != 0) {
                            const auto interval = call_start_ns - previous_servo_ns;
                            servo_interval_sum_ns += interval;
                            if (interval > servo_interval_max_ns) servo_interval_max_ns = interval;
                        }
                        previous_servo_ns = call_start_ns;
                        ++servo_calls;
                        if (servo_ret != 0) {
                            fault.set("SERVO_J_FAILED");
                            break;
                        }
                        double target_error = 0.0;
                        for (int i = 0; i < 6; ++i) {
                            target_error = std::max(target_error, std::abs(last_target[i] - playback_target[i]));
                        }
                        if (playback_elapsed_s >= playback_duration_sec && target_error < 1e-4) {
                            break;
                        }
                    }

                    if (!playback_enabled && (have_operator_zero || manual_jog_active) && last_processed_received_ns != 0) {
                        const std::uint64_t age_ns = windows_jaka::elapsed_since_ns(tick_ns, last_processed_received_ns);
                        if (latest_operator_timestamp_ns != 0 && tick_ns >= latest_operator_timestamp_ns) {
                            const auto packet_age_ns = tick_ns - latest_operator_timestamp_ns;
                            packet_age_sum_ns += packet_age_ns;
                            if (packet_age_ns > packet_age_max_ns) packet_age_max_ns = packet_age_ns;
                            ++packet_age_samples;
                        }
                        if (age_ns >= windows_jaka::kFaultAgeNs) { ++watchdog_ticks; fault.set("WATCHDOG_100MS"); break; }
                        if (age_ns >= 40'000'000ULL) ++hold_ticks;
                        // Advance the software filter and limiter on every
                        // 8 ms servo tick, not only when a 100 Hz UDP packet
                        // arrives. This keeps dt=8 ms truthful and avoids
                        // holding an old target for the extra 2-10 ms between
                        // operator packets.
                        if (age_ns < 40'000'000ULL && latest_dragging && have_latest_mapped) {
                            const auto filtered = low_pass.filter(latest_mapped);
                            const auto target = limiter.step(filtered, 0.008);
                            for (int i = 0; i < 6; ++i) {
                                const double delta = std::abs(target[i] - last_target[i]);
                                if (delta > max_target_delta[i]) max_target_delta[i] = delta;
                            }
                            last_target = target;
                        }
                        if (real_motion && manual_jog_active) {
                            for (int i = 0; i < 6; ++i) latest_mapped[i] = last_target[i] + manual_jog_delta[i];
                            const auto filtered = low_pass.filter(latest_mapped);
                            const auto target = limiter.step(filtered, 0.008);
                            for (int i = 0; i < 6; ++i) {
                                const double delta = std::abs(target[i] - last_target[i]);
                                if (delta > max_target_delta[i]) max_target_delta[i] = delta;
                            }
                            last_target = target;
                        }
                        if (have_latest_mapped) {
                            for (int i = 0; i < 6; ++i) {
                                const double error = std::abs(latest_mapped[i] - last_target[i]);
                                if (error > max_command_error[i]) max_command_error[i] = error;
                            }
                        }
                        if (real_motion) {
                            JointValue command{};
                            for (int i = 0; i < 6; ++i) {
                                command.jVal[i] = last_target[i];
                            }
                            const auto call_start_ns = windows_jaka::monotonic_ns();
                            const int servo_ret = robot.servo_j(&command, ABS, 1);
                            const auto call_end_ns = windows_jaka::monotonic_ns();
                            const auto call_duration_ns = call_end_ns - call_start_ns;
                            if (call_duration_ns > max_servo_call_ns) max_servo_call_ns = call_duration_ns;
                            if (previous_servo_ns != 0) {
                                const auto interval = call_start_ns - previous_servo_ns;
                                servo_interval_sum_ns += interval;
                                if (interval > servo_interval_max_ns) servo_interval_max_ns = interval;
                            }
                            previous_servo_ns = call_start_ns;
                            ++servo_calls;
                            status_servo_error = servo_ret;
                            if (servo_ret != 0) { fault.set("SERVO_J_FAILED"); break; }
                        }
                    }
                    if (record.is_open() && record_ret == 0) {
                        windows_jaka::JointArray recorded{};
                        for (int i = 0; i < 6; ++i) recorded[i] = record_joints.jVal[i];
                        const std::uint64_t time_ms = (tick_ns - start_ns) / 1'000'000ULL;
                        windows_jaka::write_trajectory_point(record, time_ms, recorded);
                    }
                    ++status_tick;
                    if (!options.status_file.empty() && status_tick % 62 == 0) {
                        if (status_tick % 124 == 0) {
                            RobotStatus_simple robot_status{};
                            BOOL dragging = FALSE;
                            const int status_ret = robot.get_robot_status_simple(&robot_status);
                            const int drag_ret = robot.is_in_drag_mode(&dragging);
                            if (status_ret == 0) {
                                status_robot_powered = robot_status.powered_on != 0;
                                status_robot_enabled = robot_status.enabled != 0;
                            }
                            if (drag_ret == 0) status_robot_dragging = dragging != 0;
                        }
                        const double elapsed_s = static_cast<double>(tick_ns - start_ns) / 1e9;
                        windows_jaka::RuntimeStatus runtime_status;
                        runtime_status.robot_id = options.robot_id;
                        runtime_status.mode = options.control_mode;
                        runtime_status.alarm = fault.get();
                        runtime_status.connected = logged_in;
                        runtime_status.powered = status_robot_powered;
                        runtime_status.enabled = status_robot_enabled;
                        runtime_status.dragging = status_robot_dragging;
                        runtime_status.valid = !fault.fault.load();
                        runtime_status.servo = servo_enabled;
                        runtime_status.sequence = last_sequence;
                        runtime_status.watchdog_ticks = watchdog_ticks;
                        runtime_status.dropped_packets = latest_dropped_packets;
                        runtime_status.login_code = status_login_ret;
                        runtime_status.servo_error = status_servo_error;
                        runtime_status.rate_hz = elapsed_s > 0.0 ? last_sequence / elapsed_s : 0.0;
                        if (last_processed_received_ns != 0 && tick_ns >= last_processed_received_ns) {
                            runtime_status.packet_age_ms =
                                static_cast<double>(tick_ns - last_processed_received_ns) / 1e6;
                        }
                        const std::uint64_t expected = last_sequence + latest_dropped_packets;
                        runtime_status.packet_loss_percent = expected > 0
                            ? 100.0 * static_cast<double>(latest_dropped_packets) /
                                  static_cast<double>(expected)
                            : 0.0;
                        std::string status_error;
                        if (!windows_jaka::write_runtime_status(options.status_file, runtime_status, status_error)) {
                            if (status_write_error_reported.empty()) {
                                status_write_error_reported = status_error;
                                std::cerr << "follower status write failed: " << status_error << "\n";
                            }
                        }
                    }

                }
            } catch (const std::exception& error) {
                fault.set(error.what());
            }

            if (record.is_open()) record.flush();
            stop.request_stop();
            control_stopping.store(true);
            windows_jaka::send_control_command(options.control_pipe, "STOP");
            if (servo_enabled) {
                abort_ret = robot.motion_abort();
                disable_ret = robot.servo_move_enable(FALSE, false);
                servo_enabled = false;
            }
            if (logged_in) {
                logout_ret = robot.login_out();
                logged_in = false;
            }
            std::uint64_t dropped = 0, invalid_sequences = 0, receive_sum = 0, receive_max = 0, receive_samples = 0;
            windows_jaka::JointSamplePacket ignored{};
            std::uint64_t ignored_ns = 0;
            mailbox.snapshot(ignored, ignored_ns, dropped, invalid_sequences, receive_sum, receive_max, receive_samples);
            const double receive_avg_ms = receive_samples > 1 ? static_cast<double>(receive_sum) / (receive_samples - 1) / 1e6 : 0.0;
            const double servo_avg_ms = servo_calls > 1 ? static_cast<double>(servo_interval_sum_ns) / (servo_calls - 1) / 1e6 : 0.0;
            const double packet_age_avg_ms = packet_age_samples > 0
                ? static_cast<double>(packet_age_sum_ns) / packet_age_samples / 1e6 : 0.0;
            std::cout << std::fixed << std::setprecision(4)
                      << "follower timing receive_avg_ms=" << receive_avg_ms
                      << " receive_max_ms=" << static_cast<double>(receive_max) / 1e6
                      << " packet_age_avg_ms=" << packet_age_avg_ms
                      << " packet_age_max_ms=" << static_cast<double>(packet_age_max_ns) / 1e6
                      << " servo_interval_avg_ms=" << servo_avg_ms
                      << " servo_interval_max_ms=" << static_cast<double>(servo_interval_max_ns) / 1e6
                      << " servo_call_max_ms=" << static_cast<double>(max_servo_call_ns) / 1e6
                      << " configure_ret=" << configure_ret
                      << " dropped_packets=" << dropped
                      << " hold_ticks=" << hold_ticks
                      << " watchdog_ticks=" << watchdog_ticks
                      << " record=" << (record_enabled ? 1 : 0)
                      << " playback=" << (playback_enabled ? 1 : 0)
                      << " playback_samples=" << playback_points.size()
                      << " max_target_delta_rad=[";
            for (int i = 0; i < 6; ++i) {
                if (i != 0) std::cout << ", ";
                std::cout << max_target_delta[i];
            }
            std::cout << "] max_command_error_rad=[";
            for (int i = 0; i < 6; ++i) {
                if (i != 0) std::cout << ", ";
                std::cout << max_command_error[i];
            }
            std::cout << "]\n";
            const std::string reason = fault.get();
            sdk_result.store((!reason.empty() && reason != "DURATION") ? 1 : 0);
        });

        if (sdk_thread.joinable()) sdk_thread.join();
        stop.request_stop();
        if (receiver_thread.joinable()) receiver_thread.join();
        if (control_thread.joinable()) control_thread.join();
        const std::string reason = fault.get();
        std::cout << "follower stop_reason=" << (reason.empty() ? "DURATION_OR_SIGNAL" : reason) << "\n";
        return sdk_result.load() == 0 ? 0 : 5;
    } catch (const std::exception& error) {
        stop.request_stop();
        if (sdk_thread.joinable()) sdk_thread.join();
        if (receiver_thread.joinable()) receiver_thread.join();
        control_stopping.store(true);
        windows_jaka::send_control_command(control_pipe_name, "STOP");
        if (control_thread.joinable()) control_thread.join();
        std::cerr << "windows_follower fatal: " << error.what() << "\n";
        return 10;
    }
}
