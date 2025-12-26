/**
 * @file velocity_smoother.hpp
 * @brief Velocity smoothing with acceleration limits
 *
 * Prevents jerky motion by limiting acceleration and deceleration rates.
 * Critical for:
 * - Motor longevity (reduces current spikes)
 * - Wheel slip prevention
 * - Passenger comfort (if applicable)
 * - Odometry accuracy (reduces wheel slip)
 *
 * @author MSc AI & Robotics Applicant
 */

#ifndef KINEMATIC_NAV_PKG__VELOCITY_SMOOTHER_HPP_
#define KINEMATIC_NAV_PKG__VELOCITY_SMOOTHER_HPP_

#include <algorithm>
#include <cmath>
#include <chrono>

namespace kinematic_nav
{

/**
 * @brief Configuration for velocity smoother
 */
struct VelocitySmootherConfig
{
    // Acceleration limits [m/s^2] and [rad/s^2]
    double max_linear_accel{0.5};       ///< Max forward acceleration
    double max_linear_decel{1.0};       ///< Max braking deceleration (can be higher)
    double max_angular_accel{1.5};      ///< Max angular acceleration
    double max_angular_decel{2.0};      ///< Max angular deceleration

    // Velocity limits (redundant safety)
    double max_linear_vel{0.22};
    double max_angular_vel{2.84};

    // Dead zone - don't bother smoothing tiny velocities
    double linear_dead_zone{0.001};
    double angular_dead_zone{0.001};

    // Sample time [s]
    double dt{0.02};

    VelocitySmootherConfig() = default;
};

/**
 * @brief Smoothed velocity command
 */
struct SmoothedVelocity
{
    double linear{0.0};
    double angular{0.0};

    SmoothedVelocity() = default;
    SmoothedVelocity(double v, double w) : linear(v), angular(w) {}
};

/**
 * @brief Velocity smoother with acceleration limiting
 *
 * Implements a first-order rate limiter for both linear and angular velocities.
 * Uses separate limits for acceleration (speeding up) and deceleration (slowing down).
 */
class VelocitySmoother
{
public:
    explicit VelocitySmoother(VelocitySmootherConfig config = VelocitySmootherConfig())
        : config_(std::move(config))
        , current_linear_(0.0)
        , current_angular_(0.0)
        , last_update_time_(std::chrono::steady_clock::now())
    {
    }

    /**
     * @brief Smooth a velocity command
     *
     * @param target_linear Desired linear velocity [m/s]
     * @param target_angular Desired angular velocity [rad/s]
     * @return SmoothedVelocity Rate-limited velocity command
     */
    [[nodiscard]] SmoothedVelocity smooth(double target_linear, double target_angular)
    {
        // Compute actual dt from wall clock for robustness
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - last_update_time_).count();
        last_update_time_ = now;

        // Use configured dt if measured dt is unreasonable
        const double effective_dt = (dt > 0.0 && dt < 1.0) ? dt : config_.dt;

        // Apply dead zone
        if (std::abs(target_linear) < config_.linear_dead_zone) {
            target_linear = 0.0;
        }
        if (std::abs(target_angular) < config_.angular_dead_zone) {
            target_angular = 0.0;
        }

        // Clamp targets to velocity limits
        target_linear = std::clamp(target_linear, -config_.max_linear_vel, config_.max_linear_vel);
        target_angular = std::clamp(target_angular, -config_.max_angular_vel, config_.max_angular_vel);

        // Smooth linear velocity
        current_linear_ = smoothValue(
            current_linear_,
            target_linear,
            config_.max_linear_accel,
            config_.max_linear_decel,
            effective_dt);

        // Smooth angular velocity
        current_angular_ = smoothValue(
            current_angular_,
            target_angular,
            config_.max_angular_accel,
            config_.max_angular_decel,
            effective_dt);

        return SmoothedVelocity(current_linear_, current_angular_);
    }

    /**
     * @brief Emergency stop - immediately zero velocities
     *
     * Bypasses smoothing for safety-critical situations.
     */
    void emergencyStop()
    {
        current_linear_ = 0.0;
        current_angular_ = 0.0;
    }

    /**
     * @brief Gradual stop - smoothly decelerate to zero
     *
     * @return SmoothedVelocity Current smoothed velocity (approaching zero)
     */
    [[nodiscard]] SmoothedVelocity gradualStop()
    {
        return smooth(0.0, 0.0);
    }

    /**
     * @brief Reset smoother state
     */
    void reset()
    {
        current_linear_ = 0.0;
        current_angular_ = 0.0;
        last_update_time_ = std::chrono::steady_clock::now();
    }

    /**
     * @brief Check if robot is currently stopped
     */
    [[nodiscard]] bool isStopped() const
    {
        return std::abs(current_linear_) < config_.linear_dead_zone &&
               std::abs(current_angular_) < config_.angular_dead_zone;
    }

    // State access
    [[nodiscard]] double getCurrentLinear() const { return current_linear_; }
    [[nodiscard]] double getCurrentAngular() const { return current_angular_; }
    [[nodiscard]] const VelocitySmootherConfig& getConfig() const { return config_; }

    void setConfig(const VelocitySmootherConfig& config)
    {
        config_ = config;
    }

private:
    /**
     * @brief Apply rate limiting to a single value
     */
    [[nodiscard]] static double smoothValue(
        double current,
        double target,
        double max_accel,
        double max_decel,
        double dt)
    {
        const double diff = target - current;

        if (std::abs(diff) < 1e-6) {
            return target;  // Already at target
        }

        // Determine if accelerating or decelerating
        // Accelerating: moving away from zero toward target
        // Decelerating: moving toward zero
        const bool accelerating = (std::abs(target) > std::abs(current)) ||
                                  (std::signbit(target) != std::signbit(current) && target != 0.0);

        const double max_change = (accelerating ? max_accel : max_decel) * dt;

        if (std::abs(diff) <= max_change) {
            return target;  // Can reach target in this step
        }

        // Apply rate limit
        return current + std::copysign(max_change, diff);
    }

    VelocitySmootherConfig config_;
    double current_linear_;
    double current_angular_;
    std::chrono::steady_clock::time_point last_update_time_;
};

}  // namespace kinematic_nav

#endif  // KINEMATIC_NAV_PKG__VELOCITY_SMOOTHER_HPP_
