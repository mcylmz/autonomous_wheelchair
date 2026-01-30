/**
 * @file test_integration.cpp
 * @brief Integration tests for wheelchair system
 *
 * Tests end-to-end functionality, component interaction, and fault scenarios.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "wheelchair_navigation/ChairController.h"
#include "wheelchair_navigation/WheelchairDiagnostics.h"
#include "wheelchair_navigation/MotorMonitor.h"
#include "wheelchair_navigation/EmergencyStop.h"
#include <std_msgs/Bool.h>
#include <thread>
#include <chrono>

class WheelchairIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "test_wheelchair_integration");
        }
        nh_ = std::make_shared<ros::NodeHandle>();
    }

    void TearDown() override {
        nh_.reset();
    }

    std::shared_ptr<ros::NodeHandle> nh_;
};

/**
 * Integration Test: Emergency stop workflow
 */
TEST_F(WheelchairIntegrationTest, EmergencyStopWorkflow) {
    ChairController controller(*nh_);

    // Initial state
    EXPECT_FALSE(controller.isEmergencyStopActive());
    EXPECT_EQ(controller.getState(), ControlState::DISCONNECTED);

    // Trigger emergency stop
    WheelchairError error = controller.triggerEmergencyStop("integration_test");
    EXPECT_EQ(error, WheelchairError::SUCCESS);
    EXPECT_TRUE(controller.isEmergencyStopActive());

    // Try to set motor command (should fail)
    wheelchair_navigation::MotorReference ref;
    ref.ref1 = 100;
    ref.ref2 = 100;
    error = controller.setReference(ref, true);
    EXPECT_EQ(error, WheelchairError::EMERGENCY_STOP_ACTIVE);

    // Release emergency stop
    error = controller.releaseEmergencyStop();
    EXPECT_EQ(error, WheelchairError::SUCCESS);
    EXPECT_FALSE(controller.isEmergencyStopActive());
}

/**
 * Integration Test: Diagnostics system
 */
TEST_F(WheelchairIntegrationTest, DiagnosticsSystem) {
    // Create diagnostics instance
    WheelchairDiagnostics diagnostics(*nh_);

    // Create mock motor monitor data
    wheelchair_navigation::MotorMonitor monitor;
    monitor.v_batt = 24.0;
    monitor.temp1 = 45;
    monitor.temp2 = 45;
    monitor.amps1 = 10.0;
    monitor.amps2 = 10.0;
    monitor.faultFlags = 0;
    monitor.rpm1 = 500;
    monitor.rpm2 = 500;
    monitor.encoder1 = 10000;
    monitor.encoder2 = 10000;
    monitor.cmd1 = 500;
    monitor.cmd2 = 500;
    monitor.v_int = 24.0;
    monitor.mode = "Speed";

    // Update diagnostics
    EXPECT_NO_THROW({
        diagnostics.updateMotorData(monitor);
        diagnostics.update();
    });

    // Test with low battery
    monitor.v_batt = 19.0;  // Below error threshold
    EXPECT_NO_THROW({
        diagnostics.updateMotorData(monitor);
        diagnostics.update();
    });

    // Test with high temperature
    monitor.temp1 = 80;  // Above error threshold
    EXPECT_NO_THROW({
        diagnostics.updateMotorData(monitor);
        diagnostics.update();
    });
}

/**
 * Integration Test: Emergency stop with diagnostics
 */
TEST_F(WheelchairIntegrationTest, EmergencyStopWithDiagnostics) {
    ChairController controller(*nh_);
    WheelchairDiagnostics diagnostics(*nh_);

    // Create emergency stop message
    wheelchair_navigation::EmergencyStop estop_msg;
    estop_msg.active = true;
    estop_msg.reason = "integration_test";
    estop_msg.timestamp = ros::Time::now();

    // Trigger emergency stop
    controller.triggerEmergencyStop("integration_test");

    // Update diagnostics
    EXPECT_NO_THROW({
        diagnostics.updateEmergencyStopStatus(estop_msg);
        diagnostics.update();
    });

    // Release emergency stop
    controller.releaseEmergencyStop();
    estop_msg.active = false;
    estop_msg.reason = "";
    estop_msg.timestamp = ros::Time::now();

    EXPECT_NO_THROW({
        diagnostics.updateEmergencyStopStatus(estop_msg);
        diagnostics.update();
    });
}

/**
 * Integration Test: State machine transitions
 */
TEST_F(WheelchairIntegrationTest, StateMachineTransitions) {
    ChairController controller(*nh_);

    // Test valid transitions
    std::vector<std::pair<std::string, ControlState>> expected_states = {
        {"initial", ControlState::DISCONNECTED},
        {"after_emergency", ControlState::EMERGENCY_STOP},
        {"after_release", ControlState::READY}
    };

    // Check initial state
    EXPECT_EQ(controller.getState(), ControlState::DISCONNECTED);

    // Trigger emergency stop
    controller.triggerEmergencyStop("test");
    EXPECT_EQ(controller.getState(), ControlState::EMERGENCY_STOP);

    // Release
    controller.releaseEmergencyStop();
    // State should be READY (if connected) or remain in another state
    ControlState final_state = controller.getState();
    EXPECT_TRUE(final_state == ControlState::READY ||
                final_state == ControlState::DISCONNECTED ||
                final_state == ControlState::FAULT);
}

/**
 * Integration Test: Multi-threaded diagnostics updates
 */
TEST_F(WheelchairIntegrationTest, MultiThreadedDiagnostics) {
    WheelchairDiagnostics diagnostics(*nh_);

    const int num_threads = 5;
    std::vector<std::thread> threads;

    // Create mock data
    wheelchair_navigation::MotorMonitor monitor;
    monitor.v_batt = 24.0;
    monitor.temp1 = 45;
    monitor.temp2 = 45;
    monitor.amps1 = 10.0;
    monitor.amps2 = 10.0;
    monitor.faultFlags = 0;

    // Launch multiple threads updating diagnostics
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&diagnostics, monitor]() {
            for (int j = 0; j < 10; ++j) {
                diagnostics.updateMotorData(monitor);
                diagnostics.update();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }

    // Wait for all threads
    for (auto& t : threads) {
        t.join();
    }

    // Should complete without crashes
    SUCCEED();
}

/**
 * Integration Test: Configuration loading
 */
TEST_F(WheelchairIntegrationTest, ConfigurationLoading) {
    // Test that parameters can be loaded
    ros::NodeHandle private_nh("~");

    // Set test parameters
    private_nh.setParam("device_path", "/dev/ttyACM0");
    private_nh.setParam("command_timeout", 0.2);
    private_nh.setParam("max_retries", 5);

    // Read back
    std::string device_path;
    double command_timeout;
    int max_retries;

    EXPECT_TRUE(private_nh.getParam("device_path", device_path));
    EXPECT_TRUE(private_nh.getParam("command_timeout", command_timeout));
    EXPECT_TRUE(private_nh.getParam("max_retries", max_retries));

    EXPECT_EQ(device_path, "/dev/ttyACM0");
    EXPECT_NEAR(command_timeout, 0.2, 0.001);
    EXPECT_EQ(max_retries, 5);
}

/**
 * Integration Test: Error code propagation
 */
TEST_F(WheelchairIntegrationTest, ErrorCodePropagation) {
    ChairController controller(*nh_);

    // Test that error codes are propagated correctly
    std::vector<WheelchairError> test_errors = {
        WheelchairError::SUCCESS,
        WheelchairError::DEVICE_CONNECTION_FAILED,
        WheelchairError::EMERGENCY_STOP_ACTIVE,
        WheelchairError::COMMAND_TIMEOUT
    };

    for (WheelchairError error : test_errors) {
        std::string error_string = ChairController::errorToString(error);
        EXPECT_FALSE(error_string.empty());
        EXPECT_NE(error_string, "Unknown error");
    }
}

// Main function
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_wheelchair_integration");

    // Run tests
    int result = RUN_ALL_TESTS();

    return result;
}
