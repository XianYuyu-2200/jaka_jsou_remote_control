#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace windows_jaka {

using JointArray = std::array<double, 6>;

inline JointArray map_relative(const JointArray& operator_position,
                               const JointArray& operator_zero,
                               const JointArray& follower_zero) {
    JointArray target{};
    for (std::size_t i = 0; i < target.size(); ++i) {
        target[i] = follower_zero[i] + operator_position[i] - operator_zero[i];
    }
    return target;
}

inline bool all_finite(const JointArray& values) {
    for (double value : values) {
        if (!std::isfinite(value)) return false;
    }
    return true;
}

class OnePoleLowPass {
public:
    explicit OnePoleLowPass(double alpha = 0.35) : alpha_(alpha) {
        if (!(alpha_ > 0.0 && alpha_ <= 1.0)) {
            throw std::invalid_argument("low-pass alpha must be in (0, 1]");
        }
    }

    void reset(const JointArray& value) {
        value_ = value;
        initialized_ = true;
    }

    JointArray filter(const JointArray& input) {
        if (!all_finite(input)) throw std::invalid_argument("non-finite filter input");
        if (!initialized_) reset(input);
        for (std::size_t i = 0; i < value_.size(); ++i) {
            value_[i] += alpha_ * (input[i] - value_[i]);
        }
        return value_;
    }

private:
    double alpha_;
    JointArray value_{};
    bool initialized_{false};
};

struct JointSafetyLimits {
    // JAKA Mini2 joint working ranges from the official JAKA 2026 product
    // selection guide (2026-06-18):
    // J1/J4/J6: +/-360 deg; J2: +/-125 deg; J3: +/-130 deg; J5: +/-120 deg.
    // The controller's configured soft limits remain authoritative and may
    // be narrower than these mechanical/catalog values.
    JointArray lower{-6.283185307179586, -2.181661564992912,
                     -2.2689280275926285, -6.283185307179586,
                     -2.0943951023931953, -6.283185307179586};
    JointArray upper{6.283185307179586, 2.181661564992912,
                     2.2689280275926285, 6.283185307179586,
                     2.0943951023931953, 6.283185307179586};
    // Keep explicit velocity/acceleration bounds, but avoid a very long
    // acceleration ramp at the 8 ms servo period.  The per-cycle limits
    // below remain the hard output slew limits.
    JointArray max_velocity{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    JointArray max_acceleration{8.0, 8.0, 8.0, 8.0, 8.0, 8.0};
    JointArray max_step{0.008, 0.008, 0.008, 0.006, 0.006, 0.006};
};

class JointLimiter {
public:
    explicit JointLimiter(JointSafetyLimits limits = {}) : limits_(limits) {
        for (std::size_t i = 0; i < 6; ++i) {
            if (!(limits_.lower[i] < limits_.upper[i]) ||
                !(limits_.max_velocity[i] > 0.0) ||
                !(limits_.max_acceleration[i] > 0.0) ||
                !(limits_.max_step[i] > 0.0)) {
                throw std::invalid_argument("invalid joint safety limits");
            }
        }
    }

    void reset(const JointArray& command) {
        if (!within_limits(command)) throw std::out_of_range("initial command outside limits");
        command_ = command;
        velocity_.fill(0.0);
        initialized_ = true;
    }

    bool within_limits(const JointArray& value) const {
        if (!all_finite(value)) return false;
        for (std::size_t i = 0; i < 6; ++i) {
            if (value[i] < limits_.lower[i] || value[i] > limits_.upper[i]) return false;
        }
        return true;
    }

    JointArray step(const JointArray& desired, double dt_seconds) {
        if (!initialized_) throw std::logic_error("joint limiter is not initialized");
        if (!(dt_seconds > 0.0) || !all_finite(desired) || !within_limits(desired)) {
            throw std::out_of_range("desired target rejected by joint safety limits");
        }

        for (std::size_t i = 0; i < 6; ++i) {
            const double position_error = desired[i] - command_[i];
            const double slew_velocity = limits_.max_step[i] / dt_seconds;
            const double velocity_limit = std::min(limits_.max_velocity[i], slew_velocity);
            const double stopping_distance =
                (velocity_[i] * velocity_[i]) / (2.0 * limits_.max_acceleration[i]);

            // Select a velocity that is fast while outside the braking
            // distance, but decelerates early enough to arrive at the target
            // without overshoot. This preserves the acceleration bound while
            // removing the long oscillatory tail of the old clamp-only rule.
            double requested_velocity = 0.0;
            if (std::abs(position_error) > 1e-12) {
                if (std::abs(position_error) <= stopping_distance) {
                    requested_velocity = std::copysign(
                        std::sqrt(2.0 * limits_.max_acceleration[i] * std::abs(position_error)),
                        position_error);
                } else {
                    requested_velocity = std::copysign(velocity_limit, position_error);
                }
            }
            const double max_dv = limits_.max_acceleration[i] * dt_seconds;
            double delta_v = requested_velocity - velocity_[i];
            if (delta_v > max_dv) delta_v = max_dv;
            if (delta_v < -max_dv) delta_v = -max_dv;
            const double next_velocity = velocity_[i] + delta_v;
            const double next_command = command_[i] + next_velocity * dt_seconds;
            if (position_error != 0.0 &&
                ((desired[i] - command_[i]) * (desired[i] - next_command) <= 0.0)) {
                command_[i] = desired[i];
                velocity_[i] = 0.0;
            } else {
                velocity_[i] = next_velocity;
                command_[i] = next_command;
            }
            if (command_[i] < limits_.lower[i] || command_[i] > limits_.upper[i]) {
                throw std::out_of_range("limited command outside joint safety limits");
            }
        }
        return command_;
    }

    const JointArray& command() const { return command_; }

private:
    JointSafetyLimits limits_;
    JointArray command_{};
    JointArray velocity_{};
    bool initialized_{false};
};

}  // namespace windows_jaka
