/**
 * @file test_obstacle_detector.cpp
 * @brief Unit tests for obstacle detector
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "kinematic_nav_pkg/obstacle_detector.hpp"

using namespace kinematic_nav;

class ObstacleDetectorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        ObstacleDetectorConfig config;
        config.emergency_stop_distance = 0.15;
        config.slow_down_distance = 0.5;
        config.detection_distance = 1.0;
        config.front_zone_half_angle = 0.5236;  // 30 degrees
        config.median_filter_size = 1;  // Disable filtering for tests
        detector_ = std::make_unique<ObstacleDetector>(config);
    }

    // Create a mock laser scan with obstacles at specified angles
    std::vector<float> createScan(
        double default_range,
        const std::vector<std::pair<int, double>>& obstacles,
        std::size_t num_readings = 360)
    {
        std::vector<float> ranges(num_readings, static_cast<float>(default_range));
        for (const auto& [index, range] : obstacles) {
            if (index >= 0 && static_cast<std::size_t>(index) < num_readings) {
                ranges[index] = static_cast<float>(range);
            }
        }
        return ranges;
    }

    std::unique_ptr<ObstacleDetector> detector_;
    static constexpr double ANGLE_MIN = -M_PI;
    static constexpr double ANGLE_INCREMENT = 2.0 * M_PI / 360.0;
};

TEST_F(ObstacleDetectorTest, ClearPath)
{
    auto ranges = createScan(5.0, {});  // No obstacles
    auto info = detector_->processScan(ranges, ANGLE_MIN, ANGLE_INCREMENT);

    EXPECT_FALSE(info.emergency_stop);
    EXPECT_FALSE(info.obstacle_detected);
    EXPECT_DOUBLE_EQ(info.velocity_scale, 1.0);
}

TEST_F(ObstacleDetectorTest, EmergencyStop)
{
    // Place obstacle at front (index 180 = 0 degrees)
    auto ranges = createScan(5.0, {{180, 0.1}});
    auto info = detector_->processScan(ranges, ANGLE_MIN, ANGLE_INCREMENT);

    EXPECT_TRUE(info.emergency_stop);
    EXPECT_TRUE(info.obstacle_detected);
    EXPECT_DOUBLE_EQ(info.velocity_scale, 0.0);
}

TEST_F(ObstacleDetectorTest, SlowDown)
{
    // Obstacle at 0.3m (between emergency and slow_down)
    auto ranges = createScan(5.0, {{180, 0.3}});
    auto info = detector_->processScan(ranges, ANGLE_MIN, ANGLE_INCREMENT);

    EXPECT_FALSE(info.emergency_stop);
    EXPECT_TRUE(info.obstacle_detected);
    EXPECT_GT(info.velocity_scale, 0.0);
    EXPECT_LT(info.velocity_scale, 1.0);
}

TEST_F(ObstacleDetectorTest, FrontZoneDetection)
{
    // Obstacle slightly left of center (within front zone)
    auto ranges = createScan(5.0, {{195, 0.3}});  // ~15 degrees left
    auto info = detector_->processScan(ranges, ANGLE_MIN, ANGLE_INCREMENT);

    EXPECT_TRUE(info.obstacle_detected);
    EXPECT_LT(info.front_distance, 1.0);
}

TEST_F(ObstacleDetectorTest, SideDetection)
{
    // Obstacle on left side (90 degrees)
    auto ranges = createScan(5.0, {{270, 0.4}});
    auto info = detector_->processScan(ranges, ANGLE_MIN, ANGLE_INCREMENT);

    EXPECT_TRUE(info.obstacle_detected);
    EXPECT_LT(info.left_distance, 1.0);
    EXPECT_GT(info.right_distance, 1.0);  // Right should be clear
}

TEST_F(ObstacleDetectorTest, AvoidanceDirectionLeft)
{
    // Obstacle on left, should avoid right
    auto ranges = createScan(5.0, {{270, 0.3}, {180, 0.4}});
    auto info = detector_->processScan(ranges, ANGLE_MIN, ANGLE_INCREMENT);

    auto direction = detector_->getAvoidanceDirection(info);
    ASSERT_TRUE(direction.has_value());
    EXPECT_LT(*direction, 0.0);  // Turn right (negative angular velocity)
}

TEST_F(ObstacleDetectorTest, AvoidanceDirectionRight)
{
    // Obstacle on right, should avoid left
    auto ranges = createScan(5.0, {{90, 0.3}, {180, 0.4}});
    auto info = detector_->processScan(ranges, ANGLE_MIN, ANGLE_INCREMENT);

    auto direction = detector_->getAvoidanceDirection(info);
    ASSERT_TRUE(direction.has_value());
    EXPECT_GT(*direction, 0.0);  // Turn left (positive angular velocity)
}

TEST_F(ObstacleDetectorTest, InvalidRangesIgnored)
{
    std::vector<float> ranges(360, 5.0f);
    ranges[180] = std::numeric_limits<float>::infinity();
    ranges[181] = std::nanf("");
    ranges[182] = -1.0f;

    auto info = detector_->processScan(ranges, ANGLE_MIN, ANGLE_INCREMENT);
    EXPECT_FALSE(info.obstacle_detected);
}

TEST_F(ObstacleDetectorTest, EmptyScan)
{
    std::vector<float> empty;
    auto info = detector_->processScan(empty, ANGLE_MIN, ANGLE_INCREMENT);

    EXPECT_FALSE(info.emergency_stop);
    EXPECT_FALSE(info.obstacle_detected);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
