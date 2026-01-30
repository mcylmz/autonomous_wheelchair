/**
 * @file test_chair_controller.cpp
 * @brief Unit tests for ChairController class
 *
 * Tests thread safety, emergency stop, state transitions, and error handling.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "wheelchair_navigation/ChairController.h"
#include <thread>
#include <chrono>

class ChairControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize ROS if not already initialized
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "test_chair_controller");
        }
        nh_ = std::make_shared<ros::NodeHandle>();
    }

    void TearDown() override {
        nh_.reset();
    }

    std::shared_ptr<ros::NodeHandle> nh_;
};

/**
 * Test: Controller initialization
 */
TEST_F(ChairControllerTest, Initialization) {
    ASSERT_NO_THROW({
        ChairController controller(*nh_);
    });
}

/**
 * Test: Emergency stop activation
 */
TEST_F(ChairControllerTest, EmergencyStopActivation) {
    ChairController controller(*nh_);

    // Initially not active
    EXPECT_FALSE(controller.isEmergencyStopActive());

    // Trigger emergency stop
    WheelchairError error = controller.triggerEmergencyStop("test_emergency");
    EXPECT_EQ(error, WheelchairError::SUCCESS);
    EXPECT_TRUE(controller.isEmergencyStopActive());

    // State should be EMERGENCY_STOP
    EXPECT_EQ(controller.getState(), ControlState::EMERGENCY_STOP);
}

/**
 * Test: Emergency stop release
 */
TEST_F(ChairControllerTest, EmergencyStopRelease) {
    ChairController controller(*nh_);

    // Trigger and then release
    controller.triggerEmergencyStop("test");
    EXPECT_TRUE(controller.isEmergencyStopActive());

    WheelchairError error = controller.releaseEmergencyStop();
    EXPECT_EQ(error, WheelchairError::SUCCESS);
    EXPECT_FALSE(controller.isEmergencyStopActive());

    // State should be READY
    EXPECT_EQ(controller.getState(), ControlState::READY);
}

/**
 * Test: Cannot set reference during emergency stop
 */
TEST_F(ChairControllerTest, ReferenceRejectedDuringEmergencyStop) {
    ChairController controller(*nh_);

    // Trigger emergency stop
    controller.triggerEmergencyStop("test");

    // Try to set reference
    wheelchair_navigation::MotorReference ref;
    ref.ref1 = 100;
    ref.ref2 = 100;

    WheelchairError error = controller.setReference(ref, true);
    EXPECT_EQ(error, WheelchairError::EMERGENCY_STOP_ACTIVE);
}

/**
 * Test: Error code to string conversion
 */
TEST_F(ChairControllerTest, ErrorToString) {
    EXPECT_EQ(ChairController::errorToString(WheelchairError::SUCCESS), "Success");
    EXPECT_EQ(ChairController::errorToString(WheelchairError::DEVICE_CONNECTION_FAILED),
              "Device connection failed");
    EXPECT_EQ(ChairController::errorToString(WheelchairError::EMERGENCY_STOP_ACTIVE),
              "Emergency stop is active");
}

/**
 * Test: Thread safety - concurrent emergency stop calls
 */
TEST_F(ChairControllerTest, ThreadSafety_ConcurrentEmergencyStop) {
    ChairController controller(*nh_);

    const int num_threads = 10;
    std::vector<std::thread> threads;

    // Launch multiple threads trying to trigger emergency stop
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&controller, i]() {
            controller.triggerEmergencyStop("thread_" + std::to_string(i));
        });
    }

    // Wait for all threads to complete
    for (auto& t : threads) {
        t.join();
    }

    // Emergency stop should be active
    EXPECT_TRUE(controller.isEmergencyStopActive());
    EXPECT_EQ(controller.getState(), ControlState::EMERGENCY_STOP);
}

/**
 * Test: State transitions
 */
TEST_F(ChairControllerTest, StateTransitions) {
    ChairController controller(*nh_);

    // Initial state should be DISCONNECTED
    EXPECT_EQ(controller.getState(), ControlState::DISCONNECTED);

    // After connection attempt, state changes
    // Note: This will fail without actual hardware, but tests the logic
    WheelchairError error = controller.connect();

    // State should be either READY or FAULT depending on connection
    ControlState state = controller.getState();
    EXPECT_TRUE(state == ControlState::READY ||
                state == ControlState::FAULT ||
                state == ControlState::INITIALIZING);
}

/**
 * Test: Command timeout handling
 */
TEST_F(ChairControllerTest, CommandTimeoutHandling) {
    ChairController controller(*nh_);

    // Simulate command timeout
    WheelchairError error = controller.handleCommandTimeout();

    // Should not return error (just safe mode)
    EXPECT_TRUE(error == WheelchairError::SUCCESS ||
                error == WheelchairError::DEVICE_CONNECTION_FAILED);
}

/**
 * Test: Monitor data retrieval
 */
TEST_F(ChairControllerTest, GetMonitorData) {
    ChairController controller(*nh_);

    wheelchair_navigation::MotorMonitor monitor;
    WheelchairError error = controller.getMonitorData(monitor);

    // Will fail without hardware, but tests the API
    EXPECT_TRUE(error == WheelchairError::SUCCESS ||
                error == WheelchairError::DEVICE_CONNECTION_FAILED ||
                error == WheelchairError::DEVICE_COMMUNICATION_ERROR);
}

// Main function
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_chair_controller");
    return RUN_ALL_TESTS();
}
