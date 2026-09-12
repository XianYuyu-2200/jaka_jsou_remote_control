#include "runtime_plan.hpp"

#include <iostream>
#include <string>

int main() {
    windows_jaka::RobotRegistry registry;
    auto make_robot = [](const std::string& id, const std::string& ip) {
        windows_jaka::RobotProfile robot;
        robot.id = id;
        robot.name = id;
        robot.ip = ip;
        robot.safe_pose_rad = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        return robot;
    };
    registry.robots.push_back(make_robot("leader", "192.168.1.101"));
    registry.robots.push_back(make_robot("follower_a", "192.168.1.102"));
    registry.robots.push_back(make_robot("follower_b", "192.168.1.103"));
    registry.robots.push_back(make_robot("unselected", "192.168.1.104"));

    windows_jaka::TeleopGroup group;
    group.id = "group_a";
    group.name = "one to two";
    group.operator_robot_id = "leader";
    group.follower_robot_ids = {"follower_a", "follower_b"};
    registry.groups.push_back(group);

    windows_jaka::TeleopRuntimePlan plan;
    std::string error;
    if (!windows_jaka::build_teleop_runtime_plan(registry, "group_a", 30101, plan, error)) {
        std::cerr << "plan failed: " << error << "\n";
        return 1;
    }
    if (plan.operator_robot.id != "leader" || plan.follower_robots.size() != 2 ||
        plan.follower_endpoints.size() != 2) {
        std::cerr << "plan shape mismatch\n";
        return 2;
    }
    if (plan.follower_endpoints[0].udp_port != 30101 ||
        plan.follower_endpoints[1].udp_port != 30102) {
        std::cerr << "UDP port allocation mismatch\n";
        return 3;
    }
    if (plan.follower_endpoints[0].control_pipe == plan.follower_endpoints[1].control_pipe) {
        std::cerr << "control pipes are not unique\n";
        return 4;
    }
    for (const auto& endpoint : plan.follower_endpoints) {
        if (endpoint.robot_id == "unselected") {
            std::cerr << "unselected robot entered plan\n";
            return 5;
        }
    }

    auto disabled = registry;
    disabled.find_robot("follower_b")->enabled = false;
    if (windows_jaka::build_teleop_runtime_plan(disabled, "group_a", 30101, plan, error)) {
        std::cerr << "disabled follower accepted\n";
        return 6;
    }

    if (windows_jaka::build_teleop_runtime_plan(registry, "missing", 30101, plan, error)) {
        std::cerr << "missing group accepted\n";
        return 7;
    }
    return 0;
}
