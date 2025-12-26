/**
 * @file obstacle_detector.hpp
 * @brief LiDAR-based obstacle detection for collision avoidance
 *
 * Production-grade obstacle detection with:
 * - Configurable detection zones (front, left, right)
 * - Distance-based velocity scaling
 * - Emergency stop capability
 * - Noise filtering with median filter
 *
 * @author MSc AI & Robotics Applicant
 */

#ifndef KINEMATIC_NAV_PKG__OBSTACLE_DETECTOR_HPP_
#define KINEMATIC_NAV_PKG__OBSTACLE_DETECTOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>
#include <optional>

namespace kinematic_nav
{

/**
 * @brief Configuration for obstacle detection zones
 */
struct ObstacleDetectorConfig
{
    // Detection distances [m]
    double emergency_stop_distance{0.15};   ///< Immediate stop if obstacle closer
    double slow_down_distance{0.5};         ///< Start reducing speed
    double detection_distance{1.0};         ///< Maximum detection range

    // Angular zones [rad] - relative to robot front (0 = straight ahead)
    double front_zone_half_angle{0.5236};   ///< ~30 degrees each side
    double side_zone_half_angle{1.0472};    ///< ~60 degrees each side

    // Filtering
    std::size_t median_filter_size{5};      ///< Window size for noise reduction
    double min_valid_range{0.05};           ///< Ignore readings below this
    double max_valid_range{10.0};           ///< Ignore readings above this

    ObstacleDetectorConfig() = default;
};

/**
 * @brief Result of obstacle detection analysis
 */
struct ObstacleInfo
{
    bool emergency_stop{false};         ///< Immediate stop required
    bool obstacle_detected{false};      ///< Any obstacle in detection zone
    double front_distance{std::numeric_limits<double>::infinity()};
    double left_distance{std::numeric_limits<double>::infinity()};
    double right_distance{std::numeric_limits<double>::infinity()};
    double velocity_scale{1.0};         ///< Suggested velocity multiplier [0, 1]

    [[nodiscard]] bool isClear() const { return !emergency_stop && !obstacle_detected; }
};

/**
 * @brief LiDAR-based obstacle detector
 *
 * Processes laser scan data to detect obstacles and compute safe velocities.
 * Uses a zoned approach: front, left, right sectors are analyzed separately.
 */
class ObstacleDetector
{
public:
    explicit ObstacleDetector(ObstacleDetectorConfig config = ObstacleDetectorConfig())
        : config_(std::move(config))
        , history_front_(config_.median_filter_size, std::numeric_limits<double>::infinity())
        , history_left_(config_.median_filter_size, std::numeric_limits<double>::infinity())
        , history_right_(config_.median_filter_size, std::numeric_limits<double>::infinity())
        , history_index_(0)
    {
    }

    /**
     * @brief Process a laser scan and detect obstacles
     *
     * @param ranges Vector of range measurements [m]
     * @param angle_min Starting angle of scan [rad]
     * @param angle_increment Angular step between readings [rad]
     * @return ObstacleInfo Detection results
     */
    [[nodiscard]] ObstacleInfo processScan(
        const std::vector<float>& ranges,
        double angle_min,
        double angle_increment)
    {
        if (ranges.empty()) {
            return ObstacleInfo{};  // No data, assume clear
        }

        double min_front = std::numeric_limits<double>::infinity();
        double min_left = std::numeric_limits<double>::infinity();
        double min_right = std::numeric_limits<double>::infinity();

        const std::size_t num_readings = ranges.size();

        for (std::size_t i = 0; i < num_readings; ++i) {
            const double range = static_cast<double>(ranges[i]);

            // Skip invalid readings
            if (!isValidRange(range)) {
                continue;
            }

            // Compute angle for this reading
            const double angle = angle_min + static_cast<double>(i) * angle_increment;
            const double normalized_angle = normalizeAngle(angle);

            // Classify into zones
            if (std::abs(normalized_angle) <= config_.front_zone_half_angle) {
                // Front zone
                min_front = std::min(min_front, range);
            } else if (normalized_angle > 0 && normalized_angle <= config_.side_zone_half_angle) {
                // Left zone
                min_left = std::min(min_left, range);
            } else if (normalized_angle < 0 && normalized_angle >= -config_.side_zone_half_angle) {
                // Right zone
                min_right = std::min(min_right, range);
            }
        }

        // Apply median filter for noise reduction
        min_front = applyMedianFilter(min_front, history_front_);
        min_left = applyMedianFilter(min_left, history_left_);
        min_right = applyMedianFilter(min_right, history_right_);
        history_index_ = (history_index_ + 1) % config_.median_filter_size;

        // Build result
        ObstacleInfo info;
        info.front_distance = min_front;
        info.left_distance = min_left;
        info.right_distance = min_right;

        // Emergency stop check (front only)
        if (min_front <= config_.emergency_stop_distance) {
            info.emergency_stop = true;
            info.obstacle_detected = true;
            info.velocity_scale = 0.0;
            return info;
        }

        // Obstacle detection
        const double min_overall = std::min({min_front, min_left, min_right});
        info.obstacle_detected = (min_overall <= config_.detection_distance);

        // Velocity scaling based on front distance
        if (min_front <= config_.slow_down_distance) {
            // Linear scaling from 1.0 at slow_down_distance to 0.1 at emergency_stop_distance
            const double range = config_.slow_down_distance - config_.emergency_stop_distance;
            const double dist_from_stop = min_front - config_.emergency_stop_distance;
            info.velocity_scale = 0.1 + 0.9 * (dist_from_stop / range);
            info.velocity_scale = std::clamp(info.velocity_scale, 0.1, 1.0);
        }

        return info;
    }

    /**
     * @brief Get suggested avoidance direction
     *
     * @param info Current obstacle info
     * @return std::optional<double> Suggested turn rate, or nullopt if no avoidance needed
     */
    [[nodiscard]] std::optional<double> getAvoidanceDirection(const ObstacleInfo& info) const
    {
        if (!info.obstacle_detected || info.front_distance > config_.slow_down_distance) {
            return std::nullopt;  // No avoidance needed
        }

        // Turn away from the closer side
        if (info.left_distance < info.right_distance) {
            // Obstacle on left, turn right (negative angular velocity)
            return -0.5;
        } else if (info.right_distance < info.left_distance) {
            // Obstacle on right, turn left (positive angular velocity)
            return 0.5;
        }

        // Obstacles equal on both sides, prefer right turn
        return -0.3;
    }

    // Configuration access
    [[nodiscard]] const ObstacleDetectorConfig& getConfig() const { return config_; }
    void setConfig(const ObstacleDetectorConfig& config) { config_ = config; }

private:
    [[nodiscard]] bool isValidRange(double range) const
    {
        return std::isfinite(range) &&
               range >= config_.min_valid_range &&
               range <= config_.max_valid_range;
    }

    [[nodiscard]] static double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    [[nodiscard]] double applyMedianFilter(double new_value, std::vector<double>& history)
    {
        history[history_index_] = new_value;

        // Compute median
        std::vector<double> sorted = history;
        std::sort(sorted.begin(), sorted.end());
        return sorted[sorted.size() / 2];
    }

    ObstacleDetectorConfig config_;
    std::vector<double> history_front_;
    std::vector<double> history_left_;
    std::vector<double> history_right_;
    std::size_t history_index_;
};

}  // namespace kinematic_nav

#endif  // KINEMATIC_NAV_PKG__OBSTACLE_DETECTOR_HPP_
