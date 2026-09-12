#include "trajectory.hpp"

#include <cmath>
#include <iostream>
#include <sstream>

namespace {
bool nearly_equal(double a, double b, double tolerance = 1e-12) {
    return std::abs(a - b) <= tolerance;
}
}

int main() {
    const std::string csv =
        "time_ms,j1,j2,j3,j4,j5,j6\n"
        "0,0,1,2,3,4,5\n"
        "100,1,3,3,3,5,7\n"
        "200,2,3,4,5,6,9\n";
    std::istringstream input(csv);
    std::vector<windows_jaka::TrajectoryPoint> points;
    std::string error;
    if (!windows_jaka::parse_trajectory_stream(input, points, error)) {
        std::cerr << "valid trajectory rejected: " << error << "\n";
        return 1;
    }
    if (points.size() != 3 || !nearly_equal(windows_jaka::trajectory_duration_seconds(points), 0.2)) {
        std::cerr << "unexpected trajectory metadata\n";
        return 2;
    }
    const auto middle = windows_jaka::sample_trajectory(points, 0.05);
    if (!nearly_equal(middle[0], 0.5) || !nearly_equal(middle[1], 2.0) ||
        !nearly_equal(middle[5], 6.0)) {
        std::cerr << "unexpected interpolation result\n";
        return 3;
    }
    const auto beyond_end = windows_jaka::sample_trajectory(points, 10.0);
    if (!nearly_equal(beyond_end[0], 2.0) || !nearly_equal(beyond_end[5], 9.0)) {
        std::cerr << "end clamp failed\n";
        return 4;
    }

    std::istringstream reversed(
        "time_ms,j1,j2,j3,j4,j5,j6\n"
        "0,0,0,0,0,0,0\n"
        "10,0,0,0,0,0,0\n"
        "9,0,0,0,0,0,0\n");
    if (windows_jaka::parse_trajectory_stream(reversed, points, error)) {
        std::cerr << "non-monotonic trajectory accepted\n";
        return 5;
    }

    std::ostringstream roundtrip;
    windows_jaka::write_trajectory_header(roundtrip);
    windows_jaka::JointArray sample{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    windows_jaka::write_trajectory_point(roundtrip, 0, sample);
    sample[0] = 0.2;
    windows_jaka::write_trajectory_point(roundtrip, 8, sample);
    std::istringstream roundtrip_input(roundtrip.str());
    if (!windows_jaka::parse_trajectory_stream(roundtrip_input, points, error) ||
        points.size() != 2 || !nearly_equal(points[1].joints[0], 0.2)) {
        std::cerr << "trajectory roundtrip failed: " << error << "\n";
        return 7;
    }

    std::istringstream missing_header(
        "time_ms,j1,j2,j3,j4,j5\n"
        "0,0,0,0,0,0\n");
    if (windows_jaka::parse_trajectory_stream(missing_header, points, error)) {
        std::cerr << "invalid header accepted\n";
        return 6;
    }
    return 0;
}
