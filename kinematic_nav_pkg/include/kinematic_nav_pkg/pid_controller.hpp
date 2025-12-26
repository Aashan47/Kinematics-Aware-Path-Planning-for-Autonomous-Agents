/**
 * @file pid_controller.hpp
 * @brief PID Controller class with anti-windup and derivative filtering
 *
 * This implementation follows control theory principles aligned with
 * Sapienza's Automatic Control syllabus, including:
 * - Linear transfer function representation
 * - Discrete-time implementation (Tustin/Euler)
 * - Anti-windup for integral saturation
 * - Derivative filtering to reduce noise amplification
 *
 * @author MSc AI & Robotics Applicant
 * @date 2024
 */

#ifndef KINEMATIC_NAV_PKG__PID_CONTROLLER_HPP_
#define KINEMATIC_NAV_PKG__PID_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace kinematic_nav
{

/**
 * @brief Configuration parameters for PID controller
 *
 * Encapsulates all tunable parameters in a single struct for clean
 * parameter passing and memory efficiency.
 */
struct PIDConfig
{
    double kp{1.0};                    ///< Proportional gain
    double ki{0.0};                    ///< Integral gain
    double kd{0.0};                    ///< Derivative gain
    double dt{0.02};                   ///< Sampling period [s]
    double output_min{-1.0};           ///< Minimum output (saturation)
    double output_max{1.0};            ///< Maximum output (saturation)
    double integral_min{-10.0};        ///< Anti-windup: min integral
    double integral_max{10.0};         ///< Anti-windup: max integral
    double derivative_filter_tau{0.1}; ///< Derivative low-pass filter time constant

    // Default constructor with reasonable defaults
    PIDConfig() = default;

    // Parameterized constructor for convenience
    PIDConfig(double p, double i, double d, double sample_time)
        : kp(p), ki(i), kd(d), dt(sample_time) {}
};

/**
 * @brief Discrete PID Controller with anti-windup
 *
 * Implements a velocity-form PID controller suitable for real-time control.
 *
 * Transfer Function Representation:
 *
 *   C(s) = Kp + Ki/s + Kd*s/(1 + tau*s)
 *
 * The derivative term includes a first-order low-pass filter to attenuate
 * high-frequency noise, which is critical for real robot applications.
 *
 * Discrete Implementation (Backward Euler):
 *
 *   u[k] = Kp*e[k] + Ki*sum(e[j]*dt) + Kd*(e[k]-e[k-1])/dt * filter
 */
class PIDController
{
public:
    /**
     * @brief Construct a new PID Controller
     * @param config Configuration parameters (passed by value, moved internally)
     */
    explicit PIDController(PIDConfig config)
        : config_(std::move(config))
        , integral_(0.0)
        , prev_error_(0.0)
        , prev_derivative_(0.0)
        , initialized_(false)
    {
        // Precompute filter coefficient for efficiency
        // Alpha = dt / (tau + dt) for first-order low-pass
        if (config_.derivative_filter_tau > 0.0) {
            alpha_ = config_.dt / (config_.derivative_filter_tau + config_.dt);
        } else {
            alpha_ = 1.0;  // No filtering
        }
    }

    // Rule of Five: Use defaults for move semantics
    PIDController(const PIDController&) = default;
    PIDController(PIDController&&) noexcept = default;
    PIDController& operator=(const PIDController&) = default;
    PIDController& operator=(PIDController&&) noexcept = default;
    ~PIDController() = default;

    /**
     * @brief Compute control output given current error
     *
     * @param error Current error (setpoint - measurement)
     * @return double Control output (saturated)
     */
    [[nodiscard]] double compute(double error)
    {
        // Handle first iteration (no previous error)
        if (!initialized_) {
            prev_error_ = error;
            initialized_ = true;
        }

        // Proportional term
        const double p_term = config_.kp * error;

        // Integral term with anti-windup (clamping)
        integral_ += error * config_.dt;
        integral_ = std::clamp(integral_, config_.integral_min, config_.integral_max);
        const double i_term = config_.ki * integral_;

        // Derivative term with low-pass filtering
        const double raw_derivative = (error - prev_error_) / config_.dt;
        const double filtered_derivative = alpha_ * raw_derivative +
                                           (1.0 - alpha_) * prev_derivative_;
        const double d_term = config_.kd * filtered_derivative;

        // Store state for next iteration
        prev_error_ = error;
        prev_derivative_ = filtered_derivative;

        // Compute total output with saturation
        const double output = p_term + i_term + d_term;
        return std::clamp(output, config_.output_min, config_.output_max);
    }

    /**
     * @brief Compute control with setpoint and measurement (alternative API)
     *
     * @param setpoint Desired value
     * @param measurement Current measured value
     * @return double Control output
     */
    [[nodiscard]] double compute(double setpoint, double measurement)
    {
        return compute(setpoint - measurement);
    }

    /**
     * @brief Reset controller state (integral and derivative history)
     */
    void reset()
    {
        integral_ = 0.0;
        prev_error_ = 0.0;
        prev_derivative_ = 0.0;
        initialized_ = false;
    }

    /**
     * @brief Update controller gains at runtime
     * @param kp New proportional gain
     * @param ki New integral gain
     * @param kd New derivative gain
     */
    void setGains(double kp, double ki, double kd)
    {
        config_.kp = kp;
        config_.ki = ki;
        config_.kd = kd;
    }

    /**
     * @brief Update output saturation limits
     */
    void setOutputLimits(double min_val, double max_val)
    {
        config_.output_min = min_val;
        config_.output_max = max_val;
    }

    // Getters for debugging/logging
    [[nodiscard]] double getIntegral() const { return integral_; }
    [[nodiscard]] double getPrevError() const { return prev_error_; }
    [[nodiscard]] const PIDConfig& getConfig() const { return config_; }

private:
    PIDConfig config_;         ///< Controller configuration
    double integral_;          ///< Accumulated integral term
    double prev_error_;        ///< Previous error for derivative
    double prev_derivative_;   ///< Filtered derivative (state)
    double alpha_;             ///< Precomputed filter coefficient
    bool initialized_;         ///< First-run flag
};

/**
 * @brief Factory function for creating PID controllers with smart pointers
 *
 * Demonstrates proper memory management using unique_ptr for ownership.
 */
[[nodiscard]] inline std::unique_ptr<PIDController>
makePIDController(double kp, double ki, double kd, double dt)
{
    PIDConfig config(kp, ki, kd, dt);
    return std::make_unique<PIDController>(std::move(config));
}

}  // namespace kinematic_nav

#endif  // KINEMATIC_NAV_PKG__PID_CONTROLLER_HPP_
