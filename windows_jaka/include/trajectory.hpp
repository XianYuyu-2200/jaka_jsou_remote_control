#pragma once

#include "joint_mapping.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ostream>
#include <string>
#include <vector>

namespace windows_jaka {

struct TrajectoryPoint {
    std::uint64_t time_ms{0};
    JointArray joints{};
};

inline std::vector<std::string> split_trajectory_csv_line(const std::string& line) {
    std::vector<std::string> columns;
    std::size_t start = 0;
    while (start <= line.size()) {
        const std::size_t comma = line.find(',', start);
        const std::size_t end = comma == std::string::npos ? line.size() : comma;
        columns.push_back(line.substr(start, end - start));
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return columns;
}

inline std::string trim_trajectory_text(std::string value) {
    const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    if (!value.empty() && static_cast<unsigned char>(value.front()) == 0xEF) {
        if (value.size() >= 3 && static_cast<unsigned char>(value[1]) == 0xBB &&
            static_cast<unsigned char>(value[2]) == 0xBF) {
            value.erase(0, 3);
        }
    }
    return value;
}

inline bool parse_trajectory_stream(std::istream& input,
                                    std::vector<TrajectoryPoint>& points,
                                    std::string& error) {
    points.clear();
    error.clear();

    std::string header;
    if (!std::getline(input, header)) {
        error = "trajectory file is empty";
        return false;
    }
    if (!header.empty() && header.back() == '\r') header.pop_back();
    const auto columns = split_trajectory_csv_line(header);
    if (columns.size() < 7) {
        error = "trajectory header must contain time_ms,j1,j2,j3,j4,j5,j6";
        return false;
    }
    if (trim_trajectory_text(columns[0]) != "time_ms") {
        error = "trajectory header is missing time_ms";
        return false;
    }
    for (int joint = 0; joint < 6; ++joint) {
        const std::string expected = "j" + std::to_string(joint + 1);
        if (trim_trajectory_text(columns[static_cast<std::size_t>(joint + 1)]) != expected) {
            error = "trajectory header is missing " + expected;
            return false;
        }
    }

    std::string line;
    std::uint64_t previous_time = 0;
    bool have_previous_time = false;
    std::size_t line_number = 1;
    while (std::getline(input, line)) {
        ++line_number;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (trim_trajectory_text(line).empty()) continue;
        const auto values = split_trajectory_csv_line(line);
        if (values.size() < 7) {
            error = "invalid trajectory row at line " + std::to_string(line_number);
            return false;
        }

        TrajectoryPoint point{};
        try {
            const unsigned long long time_ms = std::stoull(trim_trajectory_text(values[0]));
            point.time_ms = static_cast<std::uint64_t>(time_ms);
            for (int joint = 0; joint < 6; ++joint) {
                point.joints[static_cast<std::size_t>(joint)] =
                    std::stod(trim_trajectory_text(values[static_cast<std::size_t>(joint + 1)]));
            }
        } catch (...) {
            error = "invalid number at trajectory line " + std::to_string(line_number);
            return false;
        }
        if (!all_finite(point.joints)) {
            error = "non-finite joint value at trajectory line " + std::to_string(line_number);
            return false;
        }
        if (have_previous_time && point.time_ms <= previous_time) {
            error = "trajectory time must increase strictly at line " + std::to_string(line_number);
            return false;
        }
        previous_time = point.time_ms;
        have_previous_time = true;
        points.push_back(point);
    }

    if (points.size() < 2) {
        error = "trajectory must contain at least two samples";
        return false;
    }
    if (points.back().time_ms == points.front().time_ms) {
        error = "trajectory duration must be positive";
        return false;
    }
    return true;
}

inline bool load_trajectory_file(const std::string& path,
                                 std::vector<TrajectoryPoint>& points,
                                 std::string& error) {
    std::ifstream input(path);
    if (!input.is_open()) {
        points.clear();
        error = "cannot open trajectory file: " + path;
        return false;
    }
    return parse_trajectory_stream(input, points, error);
}

inline void write_trajectory_header(std::ostream& output) {
    output << "time_ms,j1,j2,j3,j4,j5,j6\n";
}

inline void write_trajectory_point(std::ostream& output,
                                   std::uint64_t time_ms,
                                   const JointArray& joints) {
    output << time_ms;
    for (double value : joints) output << ',' << std::setprecision(17) << value;
    output << '\n';
}

inline JointArray sample_trajectory(const std::vector<TrajectoryPoint>& points,
                                    double elapsed_seconds) {
    if (points.empty()) return {};
    const auto begin = points.begin();
    const auto end = points.end();
    const double first_time = static_cast<double>(begin->time_ms) / 1000.0;
    const double last_time = static_cast<double>((end - 1)->time_ms) / 1000.0;
    const double clamped_time = std::clamp(elapsed_seconds, first_time, last_time);

    const auto upper = std::upper_bound(
        begin, end, clamped_time,
        [](double value, const TrajectoryPoint& point) {
            return value < static_cast<double>(point.time_ms) / 1000.0;
        });
    if (upper == begin) return begin->joints;
    if (upper == end) return (end - 1)->joints;
    const auto lower = upper - 1;
    const double lower_time = static_cast<double>(lower->time_ms) / 1000.0;
    const double upper_time = static_cast<double>(upper->time_ms) / 1000.0;
    const double span = upper_time - lower_time;
    if (!(span > 0.0)) return lower->joints;
    const double ratio = std::clamp((clamped_time - lower_time) / span, 0.0, 1.0);

    JointArray output{};
    for (std::size_t i = 0; i < output.size(); ++i) {
        output[i] = lower->joints[i] + ratio * (upper->joints[i] - lower->joints[i]);
    }
    return output;
}

inline double trajectory_duration_seconds(const std::vector<TrajectoryPoint>& points) {
    if (points.size() < 2) return 0.0;
    return static_cast<double>(points.back().time_ms - points.front().time_ms) / 1000.0;
}

}  // namespace windows_jaka
