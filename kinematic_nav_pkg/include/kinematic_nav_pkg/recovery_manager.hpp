/**
 * @file recovery_manager.hpp
 * @brief Recovery behaviors for stuck/timeout situations
 *
 * Implements recovery strategies when navigation fails:
 * - Goal timeout detection
 * - Stuck detection (no progress)
 * - Recovery actions (back up, rotate, abort)
 *
 * @author MSc AI & Robotics Applicant
 */

#ifndef KINEMATIC_NAV_PKG__RECOVERY_MANAGER_HPP_
#define KINEMATIC_NAV_PKG__RECOVERY_MANAGER_HPP_

#include <chrono>
#include <cmath>
#include <functional>
#include <optional>
#include <string>

namespace kinematic_nav
{

/**
 * @brief Recovery behavior types
 */
enum class RecoveryAction
{
    NONE,           ///< No recovery needed
    BACK_UP,        ///< Reverse a short distance
    ROTATE_LEFT,    ///< Rotate left to clear obstacle
    ROTATE_RIGHT,   ///< Rotate right to clear obstacle
    WAIT,           ///< Wait for obstacle to clear
    ABORT           ///< Give up and notify user
};

/**
 * @brief Reason for recovery trigger
 */
enum class RecoveryReason
{
    NONE,
    GOAL_TIMEOUT,       ///< Exceeded time limit for goal
    STUCK_NO_PROGRESS,  ///< Not making progress toward goal
    OSCILLATION,        ///< Oscillating back and forth
    OBSTACLE_BLOCKED    ///< Blocked by obstacle
};

/**
 * @brief Configuration for recovery manager
 */
struct RecoveryConfig
{
    // Timeout settings
    double goal_timeout_sec{60.0};          ///< Max time to reach any goal
    double stuck_timeout_sec{5.0};          ///< Time without progress = stuck
    double progress_threshold{0.02};        ///< Min distance change to count as progress [m]

    // Oscillation detection
    std::size_t oscillation_window{10};     ///< Number of heading samples
    double oscillation_threshold{0.5};      ///< Heading variance threshold [rad^2]

    // Recovery action parameters
    double backup_distance{0.3};            ///< How far to reverse [m]
    double backup_speed{0.1};               ///< Reverse speed [m/s]
    double rotate_angle{1.57};              ///< Rotation amount (~90 deg) [rad]
    double rotate_speed{0.5};               ///< Rotation speed [rad/s]
    double wait_duration_sec{3.0};          ///< How long to wait

    // Retry limits
    std::size_t max_recovery_attempts{3};   ///< Abort after this many failures

    RecoveryConfig() = default;
};

/**
 * @brief Current recovery state
 */
struct RecoveryState
{
    bool in_recovery{false};
    RecoveryAction current_action{RecoveryAction::NONE};
    RecoveryReason trigger_reason{RecoveryReason::NONE};
    std::size_t attempt_count{0};
    std::chrono::steady_clock::time_point action_start_time;
    double action_progress{0.0};    ///< Progress through current action [0, 1]

    [[nodiscard]] bool shouldAbort() const
    {
        return current_action == RecoveryAction::ABORT;
    }
};

/**
 * @brief Recovery behavior manager
 *
 * Monitors navigation progress and triggers recovery behaviors when needed.
 * Uses a state machine to execute recovery sequences.
 */
class RecoveryManager
{
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    using Duration = std::chrono::duration<double>;

    explicit RecoveryManager(RecoveryConfig config = RecoveryConfig())
        : config_(std::move(config))
        , heading_history_(config_.oscillation_window, 0.0)
        , history_index_(0)
    {
        reset();
    }

    /**
     * @brief Start tracking a new goal
     *
     * Call this when a new goal is received.
     */
    void startGoal(double initial_distance)
    {
        reset();
        goal_start_time_ = Clock::now();
        last_progress_time_ = goal_start_time_;
        last_distance_ = initial_distance;
        best_distance_ = initial_distance;
        tracking_active_ = true;
    }

    /**
     * @brief Update with current navigation state
     *
     * @param current_distance Distance to goal [m]
     * @param current_heading Current robot heading [rad]
     * @param obstacle_blocked True if blocked by obstacle
     * @return std::optional<RecoveryAction> Action to take, or nullopt if normal operation
     */
    [[nodiscard]] std::optional<RecoveryAction> update(
        double current_distance,
        double current_heading,
        bool obstacle_blocked = false)
    {
        if (!tracking_active_) {
            return std::nullopt;
        }

        const auto now = Clock::now();

        // If already in recovery, continue executing
        if (state_.in_recovery) {
            return continueRecovery(now);
        }

        // Update heading history for oscillation detection
        heading_history_[history_index_] = current_heading;
        history_index_ = (history_index_ + 1) % config_.oscillation_window;

        // Check for goal timeout
        const double goal_elapsed = Duration(now - goal_start_time_).count();
        if (goal_elapsed > config_.goal_timeout_sec) {
            return triggerRecovery(RecoveryReason::GOAL_TIMEOUT, now);
        }

        // Check for stuck (no progress)
        if (current_distance < best_distance_ - config_.progress_threshold) {
            // Making progress
            best_distance_ = current_distance;
            last_progress_time_ = now;
        } else {
            // Not making progress
            const double stuck_elapsed = Duration(now - last_progress_time_).count();
            if (stuck_elapsed > config_.stuck_timeout_sec) {
                return triggerRecovery(RecoveryReason::STUCK_NO_PROGRESS, now);
            }
        }

        // Check for oscillation
        if (detectOscillation()) {
            return triggerRecovery(RecoveryReason::OSCILLATION, now);
        }

        // Check for obstacle blockage
        if (obstacle_blocked) {
            const double blocked_elapsed = Duration(now - last_blocked_time_).count();
            if (!was_blocked_) {
                last_blocked_time_ = now;
                was_blocked_ = true;
            } else if (blocked_elapsed > 2.0) {  // Blocked for 2+ seconds
                return triggerRecovery(RecoveryReason::OBSTACLE_BLOCKED, now);
            }
        } else {
            was_blocked_ = false;
        }

        last_distance_ = current_distance;
        return std::nullopt;
    }

    /**
     * @brief Get current recovery state
     */
    [[nodiscard]] const RecoveryState& getState() const { return state_; }

    /**
     * @brief Check if currently in recovery mode
     */
    [[nodiscard]] bool isInRecovery() const { return state_.in_recovery; }

    /**
     * @brief Mark current recovery action as complete
     */
    void completeRecoveryAction()
    {
        state_.in_recovery = false;
        state_.current_action = RecoveryAction::NONE;
        last_progress_time_ = Clock::now();  // Reset stuck timer
    }

    /**
     * @brief Get the velocity command for current recovery action
     *
     * @return std::pair<double, double> (linear_vel, angular_vel)
     */
    [[nodiscard]] std::pair<double, double> getRecoveryVelocity() const
    {
        switch (state_.current_action) {
            case RecoveryAction::BACK_UP:
                return {-config_.backup_speed, 0.0};
            case RecoveryAction::ROTATE_LEFT:
                return {0.0, config_.rotate_speed};
            case RecoveryAction::ROTATE_RIGHT:
                return {0.0, -config_.rotate_speed};
            case RecoveryAction::WAIT:
            case RecoveryAction::ABORT:
            case RecoveryAction::NONE:
            default:
                return {0.0, 0.0};
        }
    }

    /**
     * @brief Get human-readable reason string
     */
    [[nodiscard]] static std::string reasonToString(RecoveryReason reason)
    {
        switch (reason) {
            case RecoveryReason::GOAL_TIMEOUT: return "Goal timeout";
            case RecoveryReason::STUCK_NO_PROGRESS: return "Stuck (no progress)";
            case RecoveryReason::OSCILLATION: return "Oscillation detected";
            case RecoveryReason::OBSTACLE_BLOCKED: return "Blocked by obstacle";
            default: return "None";
        }
    }

    /**
     * @brief Reset all state
     */
    void reset()
    {
        state_ = RecoveryState{};
        tracking_active_ = false;
        was_blocked_ = false;
        best_distance_ = std::numeric_limits<double>::infinity();
        std::fill(heading_history_.begin(), heading_history_.end(), 0.0);
    }

    // Config access
    [[nodiscard]] const RecoveryConfig& getConfig() const { return config_; }
    void setConfig(const RecoveryConfig& config) { config_ = config; }

private:
    [[nodiscard]] bool detectOscillation() const
    {
        // Compute heading variance
        double sum = 0.0;
        double sum_sq = 0.0;

        for (double h : heading_history_) {
            sum += h;
            sum_sq += h * h;
        }

        const double n = static_cast<double>(heading_history_.size());
        const double mean = sum / n;
        const double variance = (sum_sq / n) - (mean * mean);

        return variance > config_.oscillation_threshold;
    }

    [[nodiscard]] std::optional<RecoveryAction> triggerRecovery(
        RecoveryReason reason,
        TimePoint now)
    {
        state_.attempt_count++;

        if (state_.attempt_count > config_.max_recovery_attempts) {
            state_.in_recovery = true;
            state_.current_action = RecoveryAction::ABORT;
            state_.trigger_reason = reason;
            return RecoveryAction::ABORT;
        }

        state_.in_recovery = true;
        state_.trigger_reason = reason;
        state_.action_start_time = now;
        state_.action_progress = 0.0;

        // Choose recovery action based on reason
        switch (reason) {
            case RecoveryReason::OBSTACLE_BLOCKED:
            case RecoveryReason::STUCK_NO_PROGRESS:
                state_.current_action = RecoveryAction::BACK_UP;
                break;
            case RecoveryReason::OSCILLATION:
                // Alternate left/right on successive attempts
                state_.current_action = (state_.attempt_count % 2 == 1)
                    ? RecoveryAction::ROTATE_LEFT
                    : RecoveryAction::ROTATE_RIGHT;
                break;
            case RecoveryReason::GOAL_TIMEOUT:
                state_.current_action = RecoveryAction::ABORT;
                break;
            default:
                state_.current_action = RecoveryAction::WAIT;
        }

        return state_.current_action;
    }

    [[nodiscard]] std::optional<RecoveryAction> continueRecovery(TimePoint now)
    {
        const double elapsed = Duration(now - state_.action_start_time).count();

        // Check if action is complete based on type
        bool complete = false;
        switch (state_.current_action) {
            case RecoveryAction::BACK_UP:
                // Time-based: t = d/v
                complete = elapsed >= (config_.backup_distance / config_.backup_speed);
                state_.action_progress = std::min(1.0,
                    elapsed / (config_.backup_distance / config_.backup_speed));
                break;
            case RecoveryAction::ROTATE_LEFT:
            case RecoveryAction::ROTATE_RIGHT:
                complete = elapsed >= (config_.rotate_angle / config_.rotate_speed);
                state_.action_progress = std::min(1.0,
                    elapsed / (config_.rotate_angle / config_.rotate_speed));
                break;
            case RecoveryAction::WAIT:
                complete = elapsed >= config_.wait_duration_sec;
                state_.action_progress = std::min(1.0, elapsed / config_.wait_duration_sec);
                break;
            case RecoveryAction::ABORT:
            case RecoveryAction::NONE:
                complete = true;
                break;
        }

        if (complete) {
            completeRecoveryAction();
            return std::nullopt;
        }

        return state_.current_action;
    }

    RecoveryConfig config_;
    RecoveryState state_;

    // Goal tracking
    TimePoint goal_start_time_;
    TimePoint last_progress_time_;
    TimePoint last_blocked_time_;
    double last_distance_{0.0};
    double best_distance_{std::numeric_limits<double>::infinity()};
    bool tracking_active_{false};
    bool was_blocked_{false};

    // Oscillation detection
    std::vector<double> heading_history_;
    std::size_t history_index_;
};

}  // namespace kinematic_nav

#endif  // KINEMATIC_NAV_PKG__RECOVERY_MANAGER_HPP_
