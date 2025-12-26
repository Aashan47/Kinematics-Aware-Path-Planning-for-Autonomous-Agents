/**
 * @file test_pid_controller.cpp
 * @brief Comprehensive unit tests for PID controller
 */

#include <gtest/gtest.h>
#include <cmath>
#include "kinematic_nav_pkg/pid_controller.hpp"

using namespace kinematic_nav;

class PIDControllerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        PIDConfig config;
        config.kp = 1.0;
        config.ki = 0.1;
        config.kd = 0.05;
        config.dt = 0.02;
        config.output_min = -1.0;
        config.output_max = 1.0;
        config.integral_min = -5.0;
        config.integral_max = 5.0;
        controller_ = std::make_unique<PIDController>(config);
    }

    std::unique_ptr<PIDController> controller_;
};

TEST_F(PIDControllerTest, ProportionalResponse)
{
    PIDConfig config;
    config.kp = 2.0;
    config.ki = 0.0;
    config.kd = 0.0;
    config.dt = 0.02;
    PIDController p_controller(config);

    double output = p_controller.compute(0.5);
    EXPECT_NEAR(output, 1.0, 0.01);
}

TEST_F(PIDControllerTest, IntegralAccumulation)
{
    PIDConfig config;
    config.kp = 0.0;
    config.ki = 1.0;
    config.kd = 0.0;
    config.dt = 0.1;
    PIDController i_controller(config);

    for (int i = 0; i < 10; ++i) {
        i_controller.compute(1.0);
    }
    EXPECT_NEAR(i_controller.getIntegral(), 1.0, 0.01);
}

TEST_F(PIDControllerTest, AntiWindup)
{
    PIDConfig config;
    config.kp = 0.0;
    config.ki = 10.0;
    config.kd = 0.0;
    config.dt = 0.1;
    config.integral_max = 2.0;
    config.integral_min = -2.0;
    PIDController i_controller(config);

    // Apply large error many times
    for (int i = 0; i < 100; ++i) {
        i_controller.compute(10.0);
    }
    // Integral should be clamped
    EXPECT_LE(i_controller.getIntegral(), 2.0);
}

TEST_F(PIDControllerTest, OutputSaturation)
{
    double output = controller_->compute(100.0);
    EXPECT_LE(output, 1.0);
    EXPECT_GE(output, -1.0);

    output = controller_->compute(-100.0);
    EXPECT_LE(output, 1.0);
    EXPECT_GE(output, -1.0);
}

TEST_F(PIDControllerTest, DerivativeKick)
{
    PIDConfig config;
    config.kp = 0.0;
    config.ki = 0.0;
    config.kd = 1.0;
    config.dt = 0.02;
    config.derivative_filter_tau = 0.0;  // No filter
    PIDController d_controller(config);

    d_controller.compute(0.0);  // Initialize
    double output = d_controller.compute(1.0);  // Step change

    // Derivative of step is large
    EXPECT_GT(std::abs(output), 0.0);
}

TEST_F(PIDControllerTest, DerivativeFiltering)
{
    PIDConfig config_no_filter;
    config_no_filter.kp = 0.0;
    config_no_filter.ki = 0.0;
    config_no_filter.kd = 1.0;
    config_no_filter.dt = 0.02;
    config_no_filter.derivative_filter_tau = 0.0;

    PIDConfig config_filtered;
    config_filtered.kp = 0.0;
    config_filtered.ki = 0.0;
    config_filtered.kd = 1.0;
    config_filtered.dt = 0.02;
    config_filtered.derivative_filter_tau = 0.1;

    PIDController no_filter(config_no_filter);
    PIDController filtered(config_filtered);

    no_filter.compute(0.0);
    filtered.compute(0.0);

    double out_no_filter = no_filter.compute(1.0);
    double out_filtered = filtered.compute(1.0);

    // Filtered should have smaller response to step
    EXPECT_LT(std::abs(out_filtered), std::abs(out_no_filter));
}

TEST_F(PIDControllerTest, ResetClearsState)
{
    controller_->compute(1.0);
    controller_->compute(1.0);
    controller_->reset();

    EXPECT_NEAR(controller_->getIntegral(), 0.0, 0.001);
    EXPECT_NEAR(controller_->getPrevError(), 0.0, 0.001);
}

TEST_F(PIDControllerTest, ConvergenceToZero)
{
    double state = 10.0;
    const double setpoint = 0.0;

    for (int i = 0; i < 500; ++i) {
        double error = setpoint - state;
        double control = controller_->compute(error);
        state += control * 0.02;
    }

    EXPECT_NEAR(state, setpoint, 0.1);
}

TEST_F(PIDControllerTest, GainUpdate)
{
    controller_->setGains(5.0, 0.5, 0.25);
    const auto& config = controller_->getConfig();
    EXPECT_DOUBLE_EQ(config.kp, 5.0);
    EXPECT_DOUBLE_EQ(config.ki, 0.5);
    EXPECT_DOUBLE_EQ(config.kd, 0.25);
}

TEST_F(PIDControllerTest, OutputLimitsUpdate)
{
    controller_->setOutputLimits(-0.5, 0.5);
    double output = controller_->compute(100.0);
    EXPECT_LE(output, 0.5);
}

TEST_F(PIDControllerTest, FactoryFunction)
{
    auto pid = makePIDController(1.0, 0.1, 0.01, 0.02);
    ASSERT_NE(pid, nullptr);
    double output = pid->compute(0.5);
    EXPECT_NE(output, 0.0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
