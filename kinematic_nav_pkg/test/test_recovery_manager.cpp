/**
 * @file test_recovery_manager.cpp
 * @brief Unit tests for recovery manager
 */

#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include "kinematic_nav_pkg/recovery_manager.hpp"

using namespace kinematic_nav;

class RecoveryManagerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        RecoveryConfig config;
        config.goal_timeout_sec = 1.0;  // Short for testing
        config.stuck_timeout_sec = 0.5;
        config.progress_threshold = 0.02;
        config.max_recovery_attempts = 3;
        manager_ = std::make_unique<RecoveryManager>(config);
    }

    std::unique_ptr<RecoveryManager> manager_;
};

TEST_F(RecoveryManagerTest, InitialState)
{
    EXPECT_FALSE(manager_->isInRecovery());
    EXPECT_EQ(manager_->getState().current_action, RecoveryAction::NONE);
}

TEST_F(RecoveryManagerTest, NormalProgress)
{
    manager_->startGoal(5.0);

    // Simulate progress toward goal
    auto action = manager_->update(4.5, 0.0);
    EXPECT_FALSE(action.has_value());
    EXPECT_FALSE(manager_->isInRecovery());

    action = manager_->update(4.0, 0.0);
    EXPECT_FALSE(action.has_value());
}

TEST_F(RecoveryManagerTest, StuckDetection)
{
    manager_->startGoal(5.0);

    // No progress for stuck_timeout
    for (int i = 0; i < 30; ++i) {
        auto action = manager_->update(5.0, 0.0);  // Same distance
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        if (action.has_value()) {
            EXPECT_EQ(manager_->getState().trigger_reason, RecoveryReason::STUCK_NO_PROGRESS);
            return;
        }
    }

    FAIL() << "Stuck detection should have triggered";
}

TEST_F(RecoveryManagerTest, GoalTimeout)
{
    manager_->startGoal(5.0);

    // Wait for goal timeout
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));

    auto action = manager_->update(4.0, 0.0);
    ASSERT_TRUE(action.has_value());
    EXPECT_EQ(manager_->getState().trigger_reason, RecoveryReason::GOAL_TIMEOUT);
}

TEST_F(RecoveryManagerTest, RecoveryVelocity)
{
    manager_->startGoal(5.0);

    // Force stuck
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    manager_->update(5.0, 0.0);

    EXPECT_TRUE(manager_->isInRecovery());

    auto [linear, angular] = manager_->getRecoveryVelocity();
    // Back up action should have negative linear velocity
    if (manager_->getState().current_action == RecoveryAction::BACK_UP) {
        EXPECT_LT(linear, 0.0);
    }
}

TEST_F(RecoveryManagerTest, RecoveryCompletion)
{
    manager_->startGoal(5.0);

    // Force stuck
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    manager_->update(5.0, 0.0);

    EXPECT_TRUE(manager_->isInRecovery());

    // Complete recovery
    manager_->completeRecoveryAction();
    EXPECT_FALSE(manager_->isInRecovery());
}

TEST_F(RecoveryManagerTest, MaxAttemptsAbort)
{
    manager_->startGoal(5.0);

    // Trigger multiple recovery attempts
    for (int attempt = 0; attempt < 5; ++attempt) {
        // Force stuck
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
        auto action = manager_->update(5.0, 0.0);

        if (action.has_value() && *action == RecoveryAction::ABORT) {
            EXPECT_TRUE(manager_->getState().shouldAbort());
            return;
        }

        manager_->completeRecoveryAction();
    }

    FAIL() << "Should have aborted after max attempts";
}

TEST_F(RecoveryManagerTest, Reset)
{
    manager_->startGoal(5.0);

    // Force stuck and recovery
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    manager_->update(5.0, 0.0);

    EXPECT_TRUE(manager_->isInRecovery());

    manager_->reset();

    EXPECT_FALSE(manager_->isInRecovery());
    EXPECT_EQ(manager_->getState().attempt_count, 0);
}

TEST_F(RecoveryManagerTest, ObstacleBlocked)
{
    manager_->startGoal(5.0);

    // Report obstacle blocked for extended time
    for (int i = 0; i < 150; ++i) {
        auto action = manager_->update(4.9, 0.0, true);  // blocked=true
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        if (action.has_value()) {
            EXPECT_EQ(manager_->getState().trigger_reason, RecoveryReason::OBSTACLE_BLOCKED);
            return;
        }
    }

    FAIL() << "Obstacle blocked should have triggered recovery";
}

TEST_F(RecoveryManagerTest, ReasonToString)
{
    EXPECT_EQ(RecoveryManager::reasonToString(RecoveryReason::GOAL_TIMEOUT), "Goal timeout");
    EXPECT_EQ(RecoveryManager::reasonToString(RecoveryReason::STUCK_NO_PROGRESS), "Stuck (no progress)");
    EXPECT_EQ(RecoveryManager::reasonToString(RecoveryReason::NONE), "None");
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
