#pragma once

#include "joint_mapping.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace windows_jaka {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kRadiansToDegrees = 180.0 / kPi;

struct RobotProfile {
    std::string id;
    std::string name;
    std::string model{"JAKA Mini2"};
    std::string ip;
    bool enabled{true};
    std::string filter{"lpf"};
    double lpf_cutoff{2.5};
    double max_velocity{1.0};
    double max_acceleration{8.0};
    JointArray lower_rad{-6.283185307179586, -2.181661564992912,
                         -2.2689280275926285, -6.283185307179586,
                         -2.0943951023931953, -6.283185307179586};
    JointArray upper_rad{6.283185307179586, 2.181661564992912,
                         2.2689280275926285, 6.283185307179586,
                         2.0943951023931953, 6.283185307179586};
    JointArray direction{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    JointArray safe_pose_rad{};
};

struct TeleopGroup {
    std::string id;
    std::string name;
    std::string operator_robot_id;
    std::vector<std::string> follower_robot_ids;
};

struct RobotRegistry {
    std::vector<RobotProfile> robots;
    std::vector<TeleopGroup> groups;

    const RobotProfile* find_robot(const std::string& id) const {
        const auto found = std::find_if(robots.begin(), robots.end(),
            [&](const RobotProfile& robot) { return robot.id == id; });
        return found == robots.end() ? nullptr : &*found;
    }

    RobotProfile* find_robot(const std::string& id) {
        const auto found = std::find_if(robots.begin(), robots.end(),
            [&](const RobotProfile& robot) { return robot.id == id; });
        return found == robots.end() ? nullptr : &*found;
    }

    std::string validate() const {
        std::set<std::string> robot_ids;
        for (const auto& robot : robots) {
            if (trim_copy(robot.id).empty()) return "robot id must not be empty";
            if (!robot_ids.insert(robot.id).second) return "duplicate robot id: " + robot.id;
            if (trim_copy(robot.name).empty()) return "robot name must not be empty: " + robot.id;
            if (trim_copy(robot.ip).empty()) return "robot IP must not be empty: " + robot.id;
            if (robot.filter != "none" && robot.filter != "lpf" && robot.filter != "nlf") {
                return "invalid filter for robot " + robot.id;
            }
            if (!(robot.lpf_cutoff > 0.0) || !(robot.max_velocity > 0.0) ||
                !(robot.max_acceleration > 0.0)) {
                return "invalid servo parameters for robot " + robot.id;
            }
            if (!all_finite(robot.lower_rad) || !all_finite(robot.upper_rad) ||
                !all_finite(robot.direction) || !all_finite(robot.safe_pose_rad)) {
                return "non-finite robot values: " + robot.id;
            }
            for (int i = 0; i < 6; ++i) {
                if (!(robot.lower_rad[i] < robot.upper_rad[i])) {
                    return "invalid joint limits for robot " + robot.id;
                }
                if (std::abs(std::abs(robot.direction[i]) - 1.0) > 1e-12) {
                    return "joint direction must be +1 or -1 for robot " + robot.id;
                }
                if (robot.safe_pose_rad[i] < robot.lower_rad[i] ||
                    robot.safe_pose_rad[i] > robot.upper_rad[i]) {
                    return "safe pose outside joint limits for robot " + robot.id;
                }
            }
        }

        std::set<std::string> group_ids;
        for (const auto& group : groups) {
            if (trim_copy(group.id).empty()) return "group id must not be empty";
            if (!group_ids.insert(group.id).second) return "duplicate group id: " + group.id;
            if (trim_copy(group.name).empty()) return "group name must not be empty: " + group.id;
            if (!find_robot(group.operator_robot_id)) {
                return "operator robot not found in group " + group.id;
            }
            if (group.follower_robot_ids.empty()) {
                return "group has no follower robots: " + group.id;
            }
            std::set<std::string> followers;
            for (const auto& follower_id : group.follower_robot_ids) {
                if (follower_id == group.operator_robot_id) {
                    return "operator cannot also be a follower in group " + group.id;
                }
                if (!find_robot(follower_id)) {
                    return "follower robot not found in group " + group.id + ": " + follower_id;
                }
                if (!followers.insert(follower_id).second) {
                    return "duplicate follower robot in group " + group.id + ": " + follower_id;
                }
            }
        }
        return {};
    }

    bool load(const std::filesystem::path& path, std::string& error) {
        std::ifstream input(path);
        if (!input.is_open()) {
            error = "cannot open registry file: " + path.string();
            return false;
        }
        RobotRegistry loaded;
        std::string section;
        std::string line;
        int line_number = 0;
        while (std::getline(input, line)) {
            ++line_number;
            if (!line.empty() && line.back() == '\r') line.pop_back();
            line = trim_copy(line);
            if (line.empty() || line[0] == '#') continue;
            if (line.front() == '[' && line.back() == ']') {
                section = line.substr(1, line.size() - 2);
                const auto colon = section.find(':');
                if (colon == std::string::npos) {
                    error = "invalid section at line " + std::to_string(line_number);
                    return false;
                }
                const std::string kind = section.substr(0, colon);
                const std::string id = section.substr(colon + 1);
                if (kind == "robot") {
                    RobotProfile robot;
                    robot.id = id;
                    loaded.robots.push_back(robot);
                } else if (kind == "group") {
                    TeleopGroup group;
                    group.id = id;
                    loaded.groups.push_back(group);
                } else {
                    error = "unknown section kind at line " + std::to_string(line_number);
                    return false;
                }
                continue;
            }

            const auto equals = line.find('=');
            if (equals == std::string::npos || section.empty()) {
                error = "invalid property at line " + std::to_string(line_number);
                return false;
            }
            const std::string key = trim_copy(line.substr(0, equals));
            const std::string value = trim_copy(line.substr(equals + 1));
            const auto colon = section.find(':');
            const std::string kind = section.substr(0, colon);
            if (kind == "robot") {
                if (loaded.robots.empty()) {
                    error = "robot property before robot section";
                    return false;
                }
                if (!set_robot_property(loaded.robots.back(), key, value, error)) {
                    error += " at line " + std::to_string(line_number);
                    return false;
                }
            } else if (kind == "group") {
                if (loaded.groups.empty()) {
                    error = "group property before group section";
                    return false;
                }
                if (!set_group_property(loaded.groups.back(), key, value, error)) {
                    error += " at line " + std::to_string(line_number);
                    return false;
                }
            }
        }
        error = loaded.validate();
        if (!error.empty()) return false;
        *this = std::move(loaded);
        return true;
    }

    bool save(const std::filesystem::path& path, std::string& error) const {
        error = validate();
        if (!error.empty()) return false;
        std::ofstream output(path, std::ios::out | std::ios::trunc);
        if (!output.is_open()) {
            error = "cannot create registry file: " + path.string();
            return false;
        }
        output << "# JAKA multi-robot registry\n\n";
        for (const auto& robot : robots) {
            output << "[robot:" << robot.id << "]\n";
            output << "name=" << robot.name << "\n";
            output << "model=" << robot.model << "\n";
            output << "ip=" << robot.ip << "\n";
            output << "enabled=" << (robot.enabled ? 1 : 0) << "\n";
            output << "filter=" << robot.filter << "\n";
            output << "lpf_cutoff=" << robot.lpf_cutoff << "\n";
            output << "max_velocity=" << robot.max_velocity << "\n";
            output << "max_acceleration=" << robot.max_acceleration << "\n";
            output << "lower_deg=" << join_array(robot.lower_rad, kRadiansToDegrees) << "\n";
            output << "upper_deg=" << join_array(robot.upper_rad, kRadiansToDegrees) << "\n";
            output << "direction=" << join_array(robot.direction, 1.0) << "\n";
            output << "safe_pose_deg=" << join_array(robot.safe_pose_rad, kRadiansToDegrees) << "\n\n";
        }
        for (const auto& group : groups) {
            output << "[group:" << group.id << "]\n";
            output << "name=" << group.name << "\n";
            output << "operator=" << group.operator_robot_id << "\n";
            output << "followers=" << join_strings(group.follower_robot_ids) << "\n\n";
        }
        return true;
    }

private:
    static std::string trim_copy(const std::string& value) {
        const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
        const auto begin = std::find_if(value.begin(), value.end(), not_space);
        const auto end = std::find_if(value.rbegin(), value.rend(), not_space).base();
        return begin >= end ? std::string{} : std::string(begin, end);
    }

    static std::vector<std::string> split(const std::string& value, char delimiter) {
        std::vector<std::string> output;
        std::size_t start = 0;
        while (start <= value.size()) {
            const auto end = value.find(delimiter, start);
            output.push_back(trim_copy(value.substr(start, end == std::string::npos
                ? std::string::npos : end - start)));
            if (end == std::string::npos) break;
            start = end + 1;
        }
        return output;
    }

    static bool parse_bool(const std::string& value, bool& output) {
        if (value == "1" || value == "true" || value == "yes" || value == "on") {
            output = true;
            return true;
        }
        if (value == "0" || value == "false" || value == "no" || value == "off") {
            output = false;
            return true;
        }
        return false;
    }

    static bool parse_double(const std::string& value, double& output) {
        try {
            std::size_t consumed = 0;
            output = std::stod(value, &consumed);
            return consumed == value.size() && std::isfinite(output);
        } catch (...) {
            return false;
        }
    }

    static bool parse_array(const std::string& value, JointArray& output) {
        const auto values = split(value, ',');
        if (values.size() != output.size()) return false;
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (!parse_double(values[i], output[i])) return false;
        }
        return true;
    }

    static bool parse_scaled_array(const std::string& value, JointArray& output, double scale) {
        if (!parse_array(value, output)) return false;
        for (double& item : output) item *= scale;
        return true;
    }

    static std::string join_array(const JointArray& values, double scale) {
        std::ostringstream output;
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i != 0) output << ',';
            output << values[i] * scale;
        }
        return output.str();
    }

    static std::string join_strings(const std::vector<std::string>& values) {
        std::ostringstream output;
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i != 0) output << ',';
            output << values[i];
        }
        return output.str();
    }

    static bool set_robot_property(RobotProfile& robot, const std::string& key,
                                   const std::string& value, std::string& error) {
        if (key == "name") robot.name = value;
        else if (key == "model") robot.model = value;
        else if (key == "ip") robot.ip = value;
        else if (key == "enabled") { if (!parse_bool(value, robot.enabled)) { error = "invalid enabled"; return false; } }
        else if (key == "filter") robot.filter = value;
        else if (key == "lpf_cutoff") { if (!parse_double(value, robot.lpf_cutoff)) { error = "invalid lpf_cutoff"; return false; } }
        else if (key == "max_velocity") { if (!parse_double(value, robot.max_velocity)) { error = "invalid max_velocity"; return false; } }
        else if (key == "max_acceleration") { if (!parse_double(value, robot.max_acceleration)) { error = "invalid max_acceleration"; return false; } }
        else if (key == "lower_deg") { if (!parse_scaled_array(value, robot.lower_rad, kDegreesToRadians)) { error = "invalid lower_deg"; return false; } }
        else if (key == "upper_deg") { if (!parse_scaled_array(value, robot.upper_rad, kDegreesToRadians)) { error = "invalid upper_deg"; return false; } }
        else if (key == "direction") { if (!parse_array(value, robot.direction)) { error = "invalid direction"; return false; } }
        else if (key == "safe_pose_deg") { if (!parse_scaled_array(value, robot.safe_pose_rad, kDegreesToRadians)) { error = "invalid safe_pose_deg"; return false; } }
        else { error = "unknown robot property: " + key; return false; }
        return true;
    }

    static bool set_group_property(TeleopGroup& group, const std::string& key,
                                   const std::string& value, std::string& error) {
        if (key == "name") group.name = value;
        else if (key == "operator") group.operator_robot_id = value;
        else if (key == "followers") {
            group.follower_robot_ids = split(value, ',');
            group.follower_robot_ids.erase(
                std::remove(group.follower_robot_ids.begin(), group.follower_robot_ids.end(), std::string{}),
                group.follower_robot_ids.end());
        } else {
            error = "unknown group property: " + key;
            return false;
        }
        return true;
    }
};

}  // namespace windows_jaka
