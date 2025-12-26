/**
 * @file point_to_point_navigator.cpp
 * @brief Production-ready ROS2 Node for PID-based point-to-point navigation
 *
 * PRODUCTION FEATURES:
 * - Obstacle avoidance using LiDAR data
 * - Velocity smoothing with acceleration limits
 * - Recovery behaviors (timeout, stuck detection, oscillation)
 * - Comprehensive error handling
 * - Diagnostic publishing
 *
 * @author MSc AI & Robotics Applicant
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "kinematic_nav_pkg/pid_controller.hpp"
#include "kinematic_nav_pkg/differential_drive_kinematics.hpp"
#include "kinematic_nav_pkg/obstacle_detector.hpp"
#include "kinematic_nav_pkg/velocity_smoother.hpp"
#include "kinematic_nav_pkg/recovery_manager.hpp"

using namespace std::chrono_literals;

namespace kinematic_nav
{

/**
 * @brief Navigation state machine states
 */
enum class NavState
{
    IDLE,           ///< Waiting for goal
    ROTATING,       ///< Aligning heading to goal
    TRANSLATING,    ///< Moving toward goal
    FINAL_ROTATE,   ///< Aligning to goal orientation
    AVOIDING,       ///< Obstacle avoidance maneuver
    RECOVERING,     ///< Executing recovery behavior
    GOAL_REACHED,   ///< At goal position
    ABORTED         ///< Navigation failed
};

/**
 * @brief Convert NavState to string for logging
 */
[[nodiscard]] constexpr const char* stateToString(NavState state)
{
    switch (state) {
        case NavState::IDLE: return "IDLE";
        case NavState::ROTATING: return "ROTATING";
        case NavState::TRANSLATING: return "TRANSLATING";
        case NavState::FINAL_ROTATE: return "FINAL_ROTATE";
        case NavState::AVOIDING: return "AVOIDING";
        case NavState::RECOVERING: return "RECOVERING";
        case NavState::GOAL_REACHED: return "GOAL_REACHED";
        case NavState::ABORTED: return "ABORTED";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Production-ready Point-to-Point Navigator Node
 */
class PointToPointNavigator : public rclcpp::Node
{
public:
    PointToPointNavigator()
        : Node("point_to_point_navigator")
        , state_(NavState::IDLE)
        , goal_received_(false)
        , odom_received_(false)
        , scan_received_(false)
        , consecutive_odom_failures_(0)
        , max_odom_failures_(50)  // 1 second at 50Hz
    {
        try {
            declareParameters();
            initializeComponents();
            createInterfaces();

            RCLCPP_INFO(get_logger(),
                "Navigator initialized [PRODUCTION MODE]. Waiting for goal on /goal_pose...");
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Initialization failed: %s", e.what());
            throw;
        }
    }

    ~PointToPointNavigator() override
    {
        // Ensure robot stops on shutdown
        publishStop();
        RCLCPP_INFO(get_logger(), "Navigator shutting down, robot stopped.");
    }

private:
    //=========================================================================
    // INITIALIZATION
    //=========================================================================

    void declareParameters()
    {
        // Control gains - distance PID
        declare_parameter("distance_kp", 1.0);
        declare_parameter("distance_ki", 0.01);
        declare_parameter("distance_kd", 0.1);

        // Control gains - heading PID
        declare_parameter("heading_kp", 2.0);
        declare_parameter("heading_ki", 0.0);
        declare_parameter("heading_kd", 0.2);

        // Thresholds
        declare_parameter("distance_tolerance", 0.05);
        declare_parameter("angle_tolerance", 0.05);

        // Robot physical parameters
        declare_parameter("wheel_radius", 0.033);
        declare_parameter("wheel_separation", 0.16);
        declare_parameter("max_linear_vel", 0.22);
        declare_parameter("max_angular_vel", 2.84);

        // Acceleration limits
        declare_parameter("max_linear_accel", 0.5);
        declare_parameter("max_angular_accel", 1.5);

        // Obstacle avoidance
        declare_parameter("emergency_stop_distance", 0.15);
        declare_parameter("slow_down_distance", 0.5);
        declare_parameter("detection_distance", 1.0);

        // Recovery
        declare_parameter("goal_timeout_sec", 60.0);
        declare_parameter("stuck_timeout_sec", 5.0);
        declare_parameter("max_recovery_attempts", 3);

        // Control loop rate
        declare_parameter("control_rate", 50.0);
    }

    void initializeComponents()
    {
        const double control_rate = get_parameter("control_rate").as_double();
        const double dt = 1.0 / control_rate;

        // Initialize PID controllers
        initializePIDControllers(dt);

        // Initialize kinematics
        RobotParams robot_params;
        robot_params.wheel_radius = get_parameter("wheel_radius").as_double();
        robot_params.wheel_separation = get_parameter("wheel_separation").as_double();
        robot_params.max_linear_vel = get_parameter("max_linear_vel").as_double();
        robot_params.max_angular_vel = get_parameter("max_angular_vel").as_double();
        kinematics_ = std::make_unique<DifferentialDriveKinematics>(robot_params);

        // Initialize obstacle detector
        ObstacleDetectorConfig obs_config;
        obs_config.emergency_stop_distance = get_parameter("emergency_stop_distance").as_double();
        obs_config.slow_down_distance = get_parameter("slow_down_distance").as_double();
        obs_config.detection_distance = get_parameter("detection_distance").as_double();
        obstacle_detector_ = std::make_unique<ObstacleDetector>(obs_config);

        // Initialize velocity smoother
        VelocitySmootherConfig vel_config;
        vel_config.max_linear_accel = get_parameter("max_linear_accel").as_double();
        vel_config.max_angular_accel = get_parameter("max_angular_accel").as_double();
        vel_config.max_linear_vel = robot_params.max_linear_vel;
        vel_config.max_angular_vel = robot_params.max_angular_vel;
        vel_config.dt = dt;
        velocity_smoother_ = std::make_unique<VelocitySmoother>(vel_config);

        // Initialize recovery manager
        RecoveryConfig rec_config;
        rec_config.goal_timeout_sec = get_parameter("goal_timeout_sec").as_double();
        rec_config.stuck_timeout_sec = get_parameter("stuck_timeout_sec").as_double();
        rec_config.max_recovery_attempts =
            static_cast<std::size_t>(get_parameter("max_recovery_attempts").as_int());
        recovery_manager_ = std::make_unique<RecoveryManager>(rec_config);

        RCLCPP_INFO(get_logger(), "All components initialized with dt=%.3fs", dt);
    }

    void initializePIDControllers(double dt)
    {
        // Distance controller
        PIDConfig dist_config;
        dist_config.kp = get_parameter("distance_kp").as_double();
        dist_config.ki = get_parameter("distance_ki").as_double();
        dist_config.kd = get_parameter("distance_kd").as_double();
        dist_config.dt = dt;
        dist_config.output_min = -get_parameter("max_linear_vel").as_double();
        dist_config.output_max = get_parameter("max_linear_vel").as_double();
        distance_pid_ = std::make_unique<PIDController>(dist_config);

        // Heading controller
        PIDConfig head_config;
        head_config.kp = get_parameter("heading_kp").as_double();
        head_config.ki = get_parameter("heading_ki").as_double();
        head_config.kd = get_parameter("heading_kd").as_double();
        head_config.dt = dt;
        head_config.output_min = -get_parameter("max_angular_vel").as_double();
        head_config.output_max = get_parameter("max_angular_vel").as_double();
        heading_pid_ = std::make_unique<PIDController>(head_config);
    }

    void createInterfaces()
    {
        // Publishers
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>("/navigator/status", 10);

        // Subscribers with QoS for reliability
        rclcpp::QoS sensor_qos(10);
        sensor_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&PointToPointNavigator::odomCallback, this, std::placeholders::_1));

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&PointToPointNavigator::goalCallback, this, std::placeholders::_1));

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", sensor_qos,
            std::bind(&PointToPointNavigator::scanCallback, this, std::placeholders::_1));

        // Control loop timer
        const double control_rate = get_parameter("control_rate").as_double();
        const auto period = std::chrono::duration<double>(1.0 / control_rate);
        control_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&PointToPointNavigator::controlLoop, this));
    }

    //=========================================================================
    // CALLBACKS
    //=========================================================================

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        try {
            // Validate message
            if (!validateOdometry(msg)) {
                handleOdomFailure("Invalid odometry data");
                return;
            }

            // Extract position
            current_pose_.x = msg->pose.pose.position.x;
            current_pose_.y = msg->pose.pose.position.y;

            // Extract yaw from quaternion
            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);

            // Validate quaternion
            const double q_norm = q.length();
            if (std::abs(q_norm - 1.0) > 0.01) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Non-unit quaternion in odom (norm=%.3f), normalizing", q_norm);
                q.normalize();
            }

            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            current_pose_.theta = yaw;

            odom_received_ = true;
            consecutive_odom_failures_ = 0;

        } catch (const std::exception& e) {
            handleOdomFailure(e.what());
        }
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        try {
            // Validate goal
            if (!validateGoal(msg)) {
                RCLCPP_ERROR(get_logger(), "Invalid goal rejected");
                return;
            }

            goal_pose_.x = msg->pose.position.x;
            goal_pose_.y = msg->pose.position.y;

            // Extract goal orientation
            tf2::Quaternion q(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);
            q.normalize();

            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            goal_pose_.theta = yaw;

            // Reset controllers and state
            distance_pid_->reset();
            heading_pid_->reset();
            velocity_smoother_->reset();

            // Compute initial distance and start recovery tracking
            const double dx = goal_pose_.x - current_pose_.x;
            const double dy = goal_pose_.y - current_pose_.y;
            const double initial_distance = std::sqrt(dx * dx + dy * dy);
            recovery_manager_->startGoal(initial_distance);

            state_ = NavState::ROTATING;
            goal_received_ = true;

            RCLCPP_INFO(get_logger(),
                "Goal accepted: (%.2f, %.2f, %.2f rad), distance: %.2fm",
                goal_pose_.x, goal_pose_.y, goal_pose_.theta, initial_distance);

            publishStatus("Goal accepted, rotating to face target");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Goal processing failed: %s", e.what());
        }
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        try {
            if (msg->ranges.empty()) {
                return;
            }

            obstacle_info_ = obstacle_detector_->processScan(
                msg->ranges,
                msg->angle_min,
                msg->angle_increment);

            scan_received_ = true;

        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Scan processing error: %s", e.what());
        }
    }

    //=========================================================================
    // CONTROL LOOP
    //=========================================================================

    void controlLoop()
    {
        // Check for critical failures
        if (!odom_received_) {
            if (goal_received_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Waiting for odometry data...");
            }
            publishStop();
            return;
        }

        if (!goal_received_ || state_ == NavState::IDLE) {
            publishStop();
            return;
        }

        // Compute errors
        const double dx = goal_pose_.x - current_pose_.x;
        const double dy = goal_pose_.y - current_pose_.y;
        const double distance_error = std::sqrt(dx * dx + dy * dy);
        const double angle_to_goal = std::atan2(dy, dx);
        const double heading_error = DifferentialDriveKinematics::normalizeAngle(
            angle_to_goal - current_pose_.theta);
        const double final_heading_error = DifferentialDriveKinematics::normalizeAngle(
            goal_pose_.theta - current_pose_.theta);

        // Get tolerances
        const double dist_tol = get_parameter("distance_tolerance").as_double();
        const double angle_tol = get_parameter("angle_tolerance").as_double();

        // Check recovery manager
        auto recovery_action = recovery_manager_->update(
            distance_error,
            current_pose_.theta,
            obstacle_info_.emergency_stop);

        if (recovery_action.has_value()) {
            handleRecoveryAction(*recovery_action);
            return;
        }

        // Check for emergency stop (obstacle too close)
        if (obstacle_info_.emergency_stop && state_ != NavState::RECOVERING) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "EMERGENCY STOP: Obstacle at %.2fm", obstacle_info_.front_distance);
            velocity_smoother_->emergencyStop();
            publishStop();
            return;
        }

        // State machine
        double linear_cmd = 0.0;
        double angular_cmd = 0.0;

        switch (state_) {
            case NavState::ROTATING:
                angular_cmd = heading_pid_->compute(heading_error);
                if (std::abs(heading_error) < angle_tol) {
                    RCLCPP_INFO(get_logger(), "Heading aligned, translating...");
                    state_ = NavState::TRANSLATING;
                    distance_pid_->reset();
                    publishStatus("Translating to goal");
                }
                break;

            case NavState::TRANSLATING:
                linear_cmd = distance_pid_->compute(distance_error);
                angular_cmd = heading_pid_->compute(heading_error);

                // Apply obstacle-based velocity scaling
                linear_cmd *= obstacle_info_.velocity_scale;

                // Reduce speed on large heading error
                if (std::abs(heading_error) > 0.3) {
                    linear_cmd *= 0.5;
                }

                // Check for obstacle avoidance needed
                if (obstacle_info_.obstacle_detected &&
                    obstacle_info_.front_distance < obstacle_detector_->getConfig().slow_down_distance) {
                    auto avoid_dir = obstacle_detector_->getAvoidanceDirection(obstacle_info_);
                    if (avoid_dir.has_value()) {
                        state_ = NavState::AVOIDING;
                        avoidance_direction_ = *avoid_dir;
                        RCLCPP_INFO(get_logger(), "Obstacle detected, avoiding...");
                        publishStatus("Avoiding obstacle");
                    }
                }

                if (distance_error < dist_tol) {
                    RCLCPP_INFO(get_logger(), "Position reached, final rotation...");
                    state_ = NavState::FINAL_ROTATE;
                    heading_pid_->reset();
                    publishStatus("Final rotation");
                }
                break;

            case NavState::AVOIDING:
                // Slow forward + turn away from obstacle
                linear_cmd = 0.05;  // Creep forward
                angular_cmd = avoidance_direction_;

                // Exit avoidance when clear
                if (!obstacle_info_.obstacle_detected ||
                    obstacle_info_.front_distance > obstacle_detector_->getConfig().slow_down_distance) {
                    state_ = NavState::TRANSLATING;
                    RCLCPP_INFO(get_logger(), "Obstacle cleared, resuming navigation");
                    publishStatus("Resuming navigation");
                }
                break;

            case NavState::RECOVERING:
                {
                    auto [rec_linear, rec_angular] = recovery_manager_->getRecoveryVelocity();
                    linear_cmd = rec_linear;
                    angular_cmd = rec_angular;
                }
                break;

            case NavState::FINAL_ROTATE:
                angular_cmd = heading_pid_->compute(final_heading_error);
                if (std::abs(final_heading_error) < angle_tol) {
                    RCLCPP_INFO(get_logger(), "GOAL REACHED!");
                    state_ = NavState::GOAL_REACHED;
                    publishStatus("Goal reached");
                }
                break;

            case NavState::GOAL_REACHED:
                goal_received_ = false;
                recovery_manager_->reset();
                state_ = NavState::IDLE;
                publishStop();
                return;

            case NavState::ABORTED:
                goal_received_ = false;
                recovery_manager_->reset();
                state_ = NavState::IDLE;
                publishStop();
                publishStatus("Navigation aborted");
                return;

            case NavState::IDLE:
            default:
                break;
        }

        // Apply velocity smoothing
        auto smoothed = velocity_smoother_->smooth(linear_cmd, angular_cmd);

        // Apply kinematic limits
        Twist2D twist_cmd(smoothed.linear, smoothed.angular);
        twist_cmd = kinematics_->clampVelocity(twist_cmd);

        // Publish
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = twist_cmd.linear;
        cmd.angular.z = twist_cmd.angular;
        cmd_vel_pub_->publish(cmd);

        // Debug logging
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 500,
            "State: %s, dist: %.2f, head_err: %.2f, vel_scale: %.2f",
            stateToString(state_), distance_error, heading_error,
            obstacle_info_.velocity_scale);
    }

    //=========================================================================
    // HELPER METHODS
    //=========================================================================

    void handleRecoveryAction(RecoveryAction action)
    {
        if (action == RecoveryAction::ABORT) {
            RCLCPP_ERROR(get_logger(), "Navigation ABORTED: %s",
                RecoveryManager::reasonToString(
                    recovery_manager_->getState().trigger_reason).c_str());
            state_ = NavState::ABORTED;
            publishStop();
            publishStatus("Navigation aborted - max recovery attempts exceeded");
            return;
        }

        state_ = NavState::RECOVERING;
        RCLCPP_WARN(get_logger(), "Recovery triggered: %s, action: %d",
            RecoveryManager::reasonToString(
                recovery_manager_->getState().trigger_reason).c_str(),
            static_cast<int>(action));
        publishStatus("Executing recovery behavior");
    }

    [[nodiscard]] bool validateOdometry(const nav_msgs::msg::Odometry::SharedPtr& msg) const
    {
        // Check for NaN/Inf in position
        if (!std::isfinite(msg->pose.pose.position.x) ||
            !std::isfinite(msg->pose.pose.position.y)) {
            return false;
        }

        // Check quaternion components are finite
        if (!std::isfinite(msg->pose.pose.orientation.x) ||
            !std::isfinite(msg->pose.pose.orientation.y) ||
            !std::isfinite(msg->pose.pose.orientation.z) ||
            !std::isfinite(msg->pose.pose.orientation.w)) {
            return false;
        }

        return true;
    }

    [[nodiscard]] bool validateGoal(const geometry_msgs::msg::PoseStamped::SharedPtr& msg) const
    {
        // Check position is finite
        if (!std::isfinite(msg->pose.position.x) ||
            !std::isfinite(msg->pose.position.y)) {
            RCLCPP_ERROR(get_logger(), "Goal position contains NaN/Inf");
            return false;
        }

        // Check reasonable distance (not too far)
        const double dx = msg->pose.position.x - current_pose_.x;
        const double dy = msg->pose.position.y - current_pose_.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        if (dist > 100.0) {  // 100m sanity check
            RCLCPP_WARN(get_logger(), "Goal very far (%.1fm), proceeding anyway", dist);
        }

        return true;
    }

    void handleOdomFailure(const std::string& reason)
    {
        consecutive_odom_failures_++;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "Odometry failure (%zu/%zu): %s",
            consecutive_odom_failures_, max_odom_failures_, reason.c_str());

        if (consecutive_odom_failures_ >= max_odom_failures_) {
            RCLCPP_ERROR(get_logger(), "Too many odometry failures, stopping");
            publishStop();
            state_ = NavState::ABORTED;
        }
    }

    void publishStop()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
    }

    void publishStatus(const std::string& status)
    {
        std_msgs::msg::String msg;
        msg.data = status;
        status_pub_->publish(msg);
    }

    //=========================================================================
    // MEMBER VARIABLES
    //=========================================================================

    // ROS2 interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Controllers
    std::unique_ptr<PIDController> distance_pid_;
    std::unique_ptr<PIDController> heading_pid_;

    // Production components
    std::unique_ptr<DifferentialDriveKinematics> kinematics_;
    std::unique_ptr<ObstacleDetector> obstacle_detector_;
    std::unique_ptr<VelocitySmoother> velocity_smoother_;
    std::unique_ptr<RecoveryManager> recovery_manager_;

    // State
    Pose2D current_pose_;
    Pose2D goal_pose_;
    NavState state_;
    ObstacleInfo obstacle_info_;
    double avoidance_direction_{0.0};

    // Flags
    bool goal_received_;
    bool odom_received_;
    bool scan_received_;

    // Error tracking
    std::size_t consecutive_odom_failures_;
    std::size_t max_odom_failures_;
};

}  // namespace kinematic_nav

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<kinematic_nav::PointToPointNavigator>();
        RCLCPP_INFO(node->get_logger(), "Starting Production Navigator...");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Fatal error: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
