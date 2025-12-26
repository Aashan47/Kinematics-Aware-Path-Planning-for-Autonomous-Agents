/**
 * @file test_velocity_smoother.cpp
 * @brief Unit tests for velocity smoother
 */

#include <gtest/gtest.h>
#include <cmath>
#include <thread>
#include <chrono>
#include "kinematic_nav_pkg/velocity_smoother.hpp"

using namespace kinematic_nav;

class VelocitySmootherTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        VelocitySmootherConfig config;
        config.max_linear_accel = 1.0;
        config.max_linear_decel = 2.0;
        config.max_angular_accel = 2.0;
        config.max_angular_decel = 3.0;
        config.max_linear_vel = 1.0;
        config.max_angular_vel = 2.0;
        config.dt = 0.02;
        smoother_ = std::make_unique<VelocitySmoother>(config);
    }

    std::unique_ptr<VelocitySmoother> smoother_;
};

TEST_F(VelocitySmootherTest, StartsAtZero)
{
    EXPECT_DOUBLE_EQ(smoother_->getCurrentLinear(), 0.0);
    EXPECT_DOUBLE_EQ(smoother_->getCurrentAngular(), 0.0);
    EXPECT_TRUE(smoother_->isStopped());
}

TEST_F(VelocitySmootherTest, AccelerationLimited)
{
    // Request full speed instantly
    auto result = smoother_->smooth(1.0, 0.0);

    // Should be limited by acceleration
    EXPECT_LT(result.linear, 1.0);
    EXPECT_GT(result.linear, 0.0);
}

TEST_F(VelocitySmootherTest, ReachesTargetEventually)
{
    // Apply many iterations
    for (int i = 0; i < 100; ++i) {
        smoother_->smooth(0.5, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    EXPECT_NEAR(smoother_->getCurrentLinear(), 0.5, 0.05);
}

TEST_F(VelocitySmootherTest, DecelerationFaster)
{
    // First accelerate to some speed
    for (int i = 0; i < 50; ++i) {
        smoother_->smooth(0.5, 0.0);
    }
    double speed_before_stop = smoother_->getCurrentLinear();

    // Now stop
    int steps_to_stop = 0;
    while (!smoother_->isStopped() && steps_to_stop < 100) {
        smoother_->smooth(0.0, 0.0);
        steps_to_stop++;
    }

    // Deceleration should be faster than acceleration
    // So stopping from 0.5 should take fewer steps than reaching 0.5
    EXPECT_LT(steps_to_stop, 50);
}

TEST_F(VelocitySmootherTest, VelocityLimits)
{
    // Apply many iterations with excessive target
    for (int i = 0; i < 200; ++i) {
        smoother_->smooth(10.0, 10.0);
    }

    EXPECT_LE(smoother_->getCurrentLinear(), 1.0);
    EXPECT_LE(smoother_->getCurrentAngular(), 2.0);
}

TEST_F(VelocitySmootherTest, EmergencyStop)
{
    // Build up speed
    for (int i = 0; i < 50; ++i) {
        smoother_->smooth(0.5, 1.0);
    }
    EXPECT_FALSE(smoother_->isStopped());

    // Emergency stop
    smoother_->emergencyStop();

    EXPECT_TRUE(smoother_->isStopped());
    EXPECT_DOUBLE_EQ(smoother_->getCurrentLinear(), 0.0);
    EXPECT_DOUBLE_EQ(smoother_->getCurrentAngular(), 0.0);
}

TEST_F(VelocitySmootherTest, GradualStop)
{
    // Build up speed
    for (int i = 0; i < 50; ++i) {
        smoother_->smooth(0.5, 0.0);
    }

    // Gradual stop should still move toward zero
    auto result = smoother_->gradualStop();
    EXPECT_LT(result.linear, smoother_->getCurrentLinear());
}

TEST_F(VelocitySmootherTest, Reset)
{
    // Build up speed
    for (int i = 0; i < 50; ++i) {
        smoother_->smooth(0.5, 1.0);
    }
    EXPECT_FALSE(smoother_->isStopped());

    smoother_->reset();

    EXPECT_TRUE(smoother_->isStopped());
}

TEST_F(VelocitySmootherTest, AngularSmoothing)
{
    auto result = smoother_->smooth(0.0, 2.0);

    // Should be limited by angular acceleration
    EXPECT_LT(result.angular, 2.0);
    EXPECT_GT(result.angular, 0.0);
}

TEST_F(VelocitySmootherTest, DeadZone)
{
    // Tiny velocities should be zeroed
    auto result = smoother_->smooth(0.0001, 0.0001);
    EXPECT_DOUBLE_EQ(result.linear, 0.0);
    EXPECT_DOUBLE_EQ(result.angular, 0.0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
