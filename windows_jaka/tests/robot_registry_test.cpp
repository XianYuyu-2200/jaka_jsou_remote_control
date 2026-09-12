#include "robot_registry.hpp"

#include <filesystem>
#include <iostream>
#include <string>

int main() {
    windows_jaka::RobotRegistry registry;
    windows_jaka::RobotProfile leader;
    leader.id = "leader";
    leader.name = "Leader";
    leader.model = "JAKA Mini2";
    leader.ip = "192.168.1.101";
    leader.safe_pose_rad = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    registry.robots.push_back(leader);

    windows_jaka::RobotProfile follower_a = leader;
    follower_a.id = "follower_a";
    follower_a.name = "Follower A";
    follower_a.ip = "192.168.1.102";
    registry.robots.push_back(follower_a);

    windows_jaka::RobotProfile follower_b = leader;
    follower_b.id = "follower_b";
    follower_b.name = "Follower B";
    follower_b.ip = "192.168.1.103";
    registry.robots.push_back(follower_b);

    windows_jaka::TeleopGroup group;
    group.id = "group_a";
    group.name = "One leader to two followers";
    group.operator_robot_id = "leader";
    group.follower_robot_ids = {"follower_a", "follower_b"};
    registry.groups.push_back(group);

    std::string error = registry.validate();
    if (!error.empty()) {
        std::cerr << "valid registry rejected: " << error << "\n";
        return 1;
    }

    const auto path = std::filesystem::temp_directory_path() / "jaka_robot_registry_test.ini";
    if (!registry.save(path, error)) {
        std::cerr << "save failed: " << error << "\n";
        return 2;
    }

    windows_jaka::RobotRegistry loaded;
    if (!loaded.load(path, error)) {
        std::cerr << "load failed: " << error << "\n";
        return 3;
    }
    if (loaded.robots.size() != 3 || loaded.groups.size() != 1) {
        std::cerr << "registry size mismatch\n";
        return 4;
    }
    if (loaded.groups[0].operator_robot_id != "leader" ||
        loaded.groups[0].follower_robot_ids.size() != 2 ||
        loaded.groups[0].follower_robot_ids[1] != "follower_b") {
        std::cerr << "group roundtrip mismatch\n";
        return 5;
    }

    auto invalid = loaded;
    invalid.robots[1].id = "leader";
    if (invalid.validate().empty()) {
        std::cerr << "duplicate robot accepted\n";
        return 6;
    }

    invalid = loaded;
    invalid.groups[0].follower_robot_ids.push_back("missing");
    if (invalid.validate().empty()) {
        std::cerr << "missing follower accepted\n";
        return 7;
    }

    invalid = loaded;
    invalid.robots[0].safe_pose_rad[0] = 100.0;
    if (invalid.validate().empty()) {
        std::cerr << "unsafe pose accepted\n";
        return 8;
    }

    std::filesystem::remove(path);
    return 0;
}
