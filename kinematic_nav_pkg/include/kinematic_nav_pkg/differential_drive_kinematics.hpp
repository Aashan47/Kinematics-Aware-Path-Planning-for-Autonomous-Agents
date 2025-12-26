/**
 * @file differential_drive_kinematics.hpp
 * @brief Kinematic model for differential drive robots
 *
 * Implements forward and inverse kinematics as per the mathematical
 * foundations documented in kinematics_theory.tex
 *
 * @author MSc AI & Robotics Applicant
 */

#ifndef KINEMATIC_NAV_PKG__DIFFERENTIAL_DRIVE_KINEMATICS_HPP_
#define KINEMATIC_NAV_PKG__DIFFERENTIAL_DRIVE_KINEMATICS_HPP_

#include <cmath>
#include <utility>

namespace kinematic_nav
{

/**
 * @brief Physical parameters of the differential drive robot
 */
struct RobotParams
{
    double wheel_radius{0.033};    ///< Wheel radius [m] (TurtleBot3 Burger)
    double wheel_separation{0.16}; ///< Distance between wheels [m]
    double max_linear_vel{0.22};   ///< Maximum linear velocity [m/s]
    double max_angular_vel{2.84};  ///< Maximum angular velocity [rad/s]
};

/**
 * @brief Robot pose in SE(2)
 */
struct Pose2D
{
    double x{0.0};      ///< X position [m]
    double y{0.0};      ///< Y position [m]
    double theta{0.0};  ///< Orientation [rad]

    Pose2D() = default;
    Pose2D(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

/**
 * @brief Robot velocity command (unicycle model)
 */
struct Twist2D
{
    double linear{0.0};   ///< Linear velocity v [m/s]
    double angular{0.0};  ///< Angular velocity omega [rad/s]

    Twist2D() = default;
    Twist2D(double v, double w) : linear(v), angular(w) {}
};

/**
 * @brief Wheel velocities for differential drive
 */
struct WheelVelocities
{
    double left{0.0};   ///< Left wheel angular velocity [rad/s]
    double right{0.0};  ///< Right wheel angular velocity [rad/s]

    WheelVelocities() = default;
    WheelVelocities(double l, double r) : left(l), right(r) {}
};

/**
 * @brief Differential Drive Kinematics Calculator
 *
 * Provides forward and inverse kinematics transformations for
 * differential drive robots. All computations follow the standard
 * unicycle model with nonholonomic constraints.
 */
class DifferentialDriveKinematics
{
public:
    /**
     * @brief Construct kinematics calculator with robot parameters
     */
    explicit DifferentialDriveKinematics(RobotParams params = RobotParams())
        : params_(std::move(params))
    {
    }

    /**
     * @brief Forward Kinematics: Wheel velocities -> Body velocity
     *
     * Equations:
     *   v = (r/2) * (omega_R + omega_L)
     *   w = (r/L) * (omega_R - omega_L)
     *
     * @param wheels Wheel angular velocities
     * @return Twist2D Body velocity (v, omega)
     */
    [[nodiscard]] Twist2D forwardKinematics(const WheelVelocities& wheels) const
    {
        const double r = params_.wheel_radius;
        const double L = params_.wheel_separation;

        Twist2D twist;
        twist.linear = (r / 2.0) * (wheels.right + wheels.left);
        twist.angular = (r / L) * (wheels.right - wheels.left);

        return twist;
    }

    /**
     * @brief Inverse Kinematics: Body velocity -> Wheel velocities
     *
     * Equations:
     *   omega_R = (1/r) * (v + (L/2)*w)
     *   omega_L = (1/r) * (v - (L/2)*w)
     *
     * @param twist Desired body velocity
     * @return WheelVelocities Required wheel angular velocities
     */
    [[nodiscard]] WheelVelocities inverseKinematics(const Twist2D& twist) const
    {
        const double r = params_.wheel_radius;
        const double L = params_.wheel_separation;

        WheelVelocities wheels;
        wheels.right = (1.0 / r) * (twist.linear + (L / 2.0) * twist.angular);
        wheels.left = (1.0 / r) * (twist.linear - (L / 2.0) * twist.angular);

        return wheels;
    }

    /**
     * @brief Propagate pose forward using Euler integration
     *
     * @param pose Current pose
     * @param twist Velocity command
     * @param dt Time step [s]
     * @return Pose2D New pose after dt
     */
    [[nodiscard]] Pose2D propagatePose(
        const Pose2D& pose,
        const Twist2D& twist,
        double dt) const
    {
        Pose2D new_pose;

        if (std::abs(twist.angular) < 1e-6) {
            // Straight line motion (avoid division by zero)
            new_pose.x = pose.x + twist.linear * std::cos(pose.theta) * dt;
            new_pose.y = pose.y + twist.linear * std::sin(pose.theta) * dt;
            new_pose.theta = pose.theta;
        } else {
            // Arc motion (exact integration)
            const double theta_new = pose.theta + twist.angular * dt;
            new_pose.x = pose.x + (twist.linear / twist.angular) *
                         (std::sin(theta_new) - std::sin(pose.theta));
            new_pose.y = pose.y + (twist.linear / twist.angular) *
                         (std::cos(pose.theta) - std::cos(theta_new));
            new_pose.theta = normalizeAngle(theta_new);
        }

        return new_pose;
    }

    /**
     * @brief Clamp velocity command to physical limits
     */
    [[nodiscard]] Twist2D clampVelocity(const Twist2D& twist) const
    {
        Twist2D clamped;
        clamped.linear = std::clamp(twist.linear,
                                    -params_.max_linear_vel,
                                    params_.max_linear_vel);
        clamped.angular = std::clamp(twist.angular,
                                     -params_.max_angular_vel,
                                     params_.max_angular_vel);
        return clamped;
    }

    /**
     * @brief Normalize angle to [-pi, pi]
     */
    [[nodiscard]] static double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // Accessor
    [[nodiscard]] const RobotParams& getParams() const { return params_; }

private:
    RobotParams params_;
};

}  // namespace kinematic_nav

#endif  // KINEMATIC_NAV_PKG__DIFFERENTIAL_DRIVE_KINEMATICS_HPP_
