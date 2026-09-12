#pragma once

#include "robot_registry.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace windows_jaka {

struct RuntimeEndpoint {
    std::string robot_id;
    std::string robot_name;
    std::string ip;
    std::uint16_t udp_port{0};
    std::string control_pipe;
};

struct TeleopRuntimePlan {
    TeleopGroup group;
    RobotProfile operator_robot;
    std::vector<RobotProfile> follower_robots;
    std::vector<RuntimeEndpoint> follower_endpoints;
    std::string operator_control_pipe;
};

inline std::string safe_pipe_component(std::string value) {
    for (char& ch : value) {
        const unsigned char uch = static_cast<unsigned char>(ch);
        if (!std::isalnum(uch) && ch != '_' && ch != '-') ch = '_';
    }
    return value.empty() ? "unnamed" : value;
}

inline bool build_teleop_runtime_plan(const RobotRegistry& registry,
                                      const std::string& group_id,
                                      std::uint16_t base_port,
                                      TeleopRuntimePlan& plan,
                                      std::string& error) {
    plan = {};
    error.clear();
    if (base_port == 0) {
        error = "base UDP port must be non-zero";
        return false;
    }

    const std::string registry_error = registry.validate();
    if (!registry_error.empty()) {
        error = registry_error;
        return false;
    }

    const auto group_it = std::find_if(registry.groups.begin(), registry.groups.end(),
        [&](const TeleopGroup& group) { return group.id == group_id; });
    if (group_it == registry.groups.end()) {
        error = "teleop group not found: " + group_id;
        return false;
    }
    if (group_it->follower_robot_ids.size() > static_cast<std::size_t>(65535 - base_port)) {
        error = "not enough UDP ports for group: " + group_id;
        return false;
    }

    const RobotProfile* operator_robot = registry.find_robot(group_it->operator_robot_id);
    if (operator_robot == nullptr) {
        error = "operator robot not found: " + group_it->operator_robot_id;
        return false;
    }
    if (!operator_robot->enabled) {
        error = "operator robot is disabled: " + operator_robot->id;
        return false;
    }

    plan.group = *group_it;
    plan.operator_robot = *operator_robot;
    plan.operator_control_pipe = "\\\\.\\pipe\\jaka_multi_operator_" +
        safe_pipe_component(plan.group.id);

    for (std::size_t i = 0; i < group_it->follower_robot_ids.size(); ++i) {
        const RobotProfile* follower = registry.find_robot(group_it->follower_robot_ids[i]);
        if (follower == nullptr) {
            error = "follower robot not found: " + group_it->follower_robot_ids[i];
            return false;
        }
        if (!follower->enabled) {
            error = "follower robot is disabled: " + follower->id;
            return false;
        }

        RuntimeEndpoint endpoint;
        endpoint.robot_id = follower->id;
        endpoint.robot_name = follower->name;
        endpoint.ip = follower->ip;
        endpoint.udp_port = static_cast<std::uint16_t>(base_port + i);
        endpoint.control_pipe = "\\\\.\\pipe\\jaka_multi_follower_" +
            safe_pipe_component(follower->id);
        plan.follower_robots.push_back(*follower);
        plan.follower_endpoints.push_back(std::move(endpoint));
    }

    if (plan.follower_endpoints.empty()) {
        error = "teleop group has no enabled followers: " + group_id;
        return false;
    }
    return true;
}

}  // namespace windows_jaka
