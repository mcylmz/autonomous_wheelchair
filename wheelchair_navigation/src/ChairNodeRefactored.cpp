#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "wheelchair_navigation/ChairController.h"
#include "wheelchair_navigation/WheelchairDiagnostics.h"
#include "wheelchair_navigation/MotorMonitor.h"
#include "wheelchair_navigation/MotorReference.h"
#include "wheelchair_navigation/SpeedReference.h"
#include "wheelchair_navigation/ModeChange.h"
#include "wheelchair_navigation/EmergencyStop.h"
#include "wheelchair_navigation/EmergencyStopService.h"
#include "wheelchair_navigation/ChairNode.h"

// Global controller instance (will be initialized in main)
std::unique_ptr<ChairController> g_controller;
std::unique_ptr<WheelchairDiagnostics> g_diagnostics;

// Publishers
ros::Publisher g_monitorPub;
ros::Publisher g_emergencyStopPub;

// Timestamp of last command
ros::Time g_lastCommandTime;
double g_commandTimeoutThreshold;

// Message Callback Functions
void referenceCallback(const wheelchair_navigation::MotorReference::ConstPtr& msg)
{
    if (!g_controller) {
        ROS_ERROR("Controller not initialized");
        return;
    }

    g_lastCommandTime = ros::Time::now();

    WheelchairError error = g_controller->setReference(*msg, false); // false = torque mode
    if (error != WheelchairError::SUCCESS) {
        ROS_ERROR_THROTTLE(1.0, "Failed to set torque reference: %s",
                          ChairController::errorToString(error).c_str());
    }
}

void speedRefCallback(const wheelchair_navigation::SpeedReference::ConstPtr& msg)
{
    if (!g_controller) {
        ROS_ERROR("Controller not initialized");
        return;
    }

    g_lastCommandTime = ros::Time::now();

    // Convert SpeedReference to MotorReference
    wheelchair_navigation::MotorReference ref;
    ref.ref1 = msg->left;
    ref.ref2 = msg->right;

    WheelchairError error = g_controller->setReference(ref, true); // true = speed mode
    if (error != WheelchairError::SUCCESS) {
        ROS_ERROR_THROTTLE(1.0, "Failed to set speed reference: %s",
                          ChairController::errorToString(error).c_str());
    }
}

void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (!g_controller) {
        ROS_ERROR("Controller not initialized");
        return;
    }

    if (msg->data) {
        WheelchairError error = g_controller->triggerEmergencyStop("manual_trigger");
        if (error == WheelchairError::SUCCESS) {
            ROS_ERROR("Emergency stop activated via topic");

            // Publish emergency stop status
            wheelchair_navigation::EmergencyStop estop_msg;
            estop_msg.active = true;
            estop_msg.reason = "manual_trigger";
            estop_msg.timestamp = ros::Time::now();
            g_emergencyStopPub.publish(estop_msg);

            // Update diagnostics
            if (g_diagnostics) {
                g_diagnostics->updateEmergencyStopStatus(estop_msg);
            }
        }
    } else {
        WheelchairError error = g_controller->releaseEmergencyStop();
        if (error == WheelchairError::SUCCESS) {
            ROS_INFO("Emergency stop released via topic");

            // Publish emergency stop status
            wheelchair_navigation::EmergencyStop estop_msg;
            estop_msg.active = false;
            estop_msg.reason = "";
            estop_msg.timestamp = ros::Time::now();
            g_emergencyStopPub.publish(estop_msg);

            // Update diagnostics
            if (g_diagnostics) {
                g_diagnostics->updateEmergencyStopStatus(estop_msg);
            }
        }
    }
}

// Service Callback Functions
bool modeChangeCallback(wheelchair_navigation::ModeChange::Request &req,
                       wheelchair_navigation::ModeChange::Response &res)
{
    if (!g_controller) {
        ROS_ERROR("Controller not initialized");
        return false;
    }

    // Note: This is a simplified version. In a full implementation,
    // you'd want to add a proper mode change method to ChairController
    ROS_WARN("Mode change service called with mode %d - feature not fully implemented in refactored version",
             req.newMode);

    return true;
}

bool emergencyStopServiceCallback(wheelchair_navigation::EmergencyStopService::Request &req,
                                  wheelchair_navigation::EmergencyStopService::Response &res)
{
    if (!g_controller) {
        ROS_ERROR("Controller not initialized");
        res.success = false;
        res.message = "Controller not initialized";
        return true;
    }

    WheelchairError error;

    if (req.activate) {
        error = g_controller->triggerEmergencyStop(req.reason);
        res.success = (error == WheelchairError::SUCCESS);
        res.message = res.success ? "Emergency stop activated" :
                      ChairController::errorToString(error);

        if (res.success) {
            wheelchair_navigation::EmergencyStop estop_msg;
            estop_msg.active = true;
            estop_msg.reason = req.reason;
            estop_msg.timestamp = ros::Time::now();
            g_emergencyStopPub.publish(estop_msg);

            // Update diagnostics
            if (g_diagnostics) {
                g_diagnostics->updateEmergencyStopStatus(estop_msg);
            }
        }
    } else {
        error = g_controller->releaseEmergencyStop();
        res.success = (error == WheelchairError::SUCCESS);
        res.message = res.success ? "Emergency stop released" :
                      ChairController::errorToString(error);

        if (res.success) {
            wheelchair_navigation::EmergencyStop estop_msg;
            estop_msg.active = false;
            estop_msg.reason = "";
            estop_msg.timestamp = ros::Time::now();
            g_emergencyStopPub.publish(estop_msg);

            // Update diagnostics
            if (g_diagnostics) {
                g_diagnostics->updateEmergencyStopStatus(estop_msg);
            }
        }
    }

    return true;
}

// Timer Callback Functions
void monitorCallback(const ros::TimerEvent &timer_event)
{
    if (!g_controller) {
        return;
    }

    wheelchair_navigation::MotorMonitor monitor;
    WheelchairError error = g_controller->getMonitorData(monitor);

    if (error == WheelchairError::SUCCESS) {
        g_monitorPub.publish(monitor);

        // Update diagnostics with latest motor data
        if (g_diagnostics) {
            g_diagnostics->updateMotorData(monitor);
        }

        // Check for motor faults
        if (monitor.faultFlags != 0) {
            ROS_ERROR_THROTTLE(5.0, "Motor fault detected: 0x%02X", monitor.faultFlags);
            // Optionally trigger emergency stop on critical faults
            // g_controller->triggerEmergencyStop("motor_fault");
        }

        // Note: Diagnostics system now handles voltage/temperature warnings
        // Manual checks kept for immediate feedback in logs
        if (monitor.v_batt < 20.0) {
            ROS_ERROR_THROTTLE(5.0, "CRITICAL: Battery voltage: %.1fV", monitor.v_batt);
        }

        if (monitor.temp1 > 75 || monitor.temp2 > 75) {
            ROS_ERROR_THROTTLE(5.0, "CRITICAL: Motor temperature! M1: %dC, M2: %dC",
                             monitor.temp1, monitor.temp2);
        }
    }
}

void commandCallback(const ros::TimerEvent &timer_event)
{
    if (!g_controller) {
        return;
    }

    // Check for command timeout
    ros::Duration timeSinceLastCommand = ros::Time::now() - g_lastCommandTime;

    if (timeSinceLastCommand.toSec() > g_commandTimeoutThreshold) {
        WheelchairError error = g_controller->handleCommandTimeout();
        if (error != WheelchairError::SUCCESS) {
            ROS_ERROR_THROTTLE(5.0, "Failed to handle command timeout: %s",
                             ChairController::errorToString(error).c_str());
        }
    }
}

void diagnosticsCallback(const ros::TimerEvent &timer_event)
{
    if (g_diagnostics) {
        g_diagnostics->update();
    }
}

int main(int argc, char* argv[])
{
    // Initialize ROS interface
    ros::init(argc, argv, "wheelchair_node");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    // Load parameters
    private_nh.param("command_timeout", g_commandTimeoutThreshold, 0.2);
    double monitor_rate, command_rate, diagnostics_rate;
    private_nh.param("monitor_rate", monitor_rate, 50.0);      // 50 Hz = 0.02s
    private_nh.param("command_rate", command_rate, 10.0);      // 10 Hz = 0.1s
    private_nh.param("diagnostics/update_rate", diagnostics_rate, 1.0);  // 1 Hz

    ROS_INFO("Starting wheelchair node with refactored architecture");
    ROS_INFO("Command timeout: %.2f s", g_commandTimeoutThreshold);
    ROS_INFO("Monitor rate: %.1f Hz, Command rate: %.1f Hz", monitor_rate, command_rate);
    ROS_INFO("Diagnostics rate: %.1f Hz", diagnostics_rate);

    // Create controller instance
    g_controller = std::make_unique<ChairController>(private_nh);

    // Create diagnostics instance
    g_diagnostics = std::make_unique<WheelchairDiagnostics>(n);

    // Connect to device
    WheelchairError error = g_controller->connect();
    if (error != WheelchairError::SUCCESS) {
        ROS_FATAL("Failed to connect to Roboteq device: %s",
                 ChairController::errorToString(error).c_str());
        return 1;
    }

    // Initialize command timestamp
    g_lastCommandTime = ros::Time::now();

    // Setup publishers
    g_monitorPub = n.advertise<wheelchair_navigation::MotorMonitor>("motor_monitor", 10);
    g_emergencyStopPub = n.advertise<wheelchair_navigation::EmergencyStop>("emergency_stop_status", 10);

    // Setup subscribers
    ros::Subscriber referenceSub = n.subscribe("motor_ref", 10, referenceCallback);
    ros::Subscriber speedRef = n.subscribe("speed_ref", 10, speedRefCallback);
    ros::Subscriber emergencyStopSub = n.subscribe("emergency_stop", 10, emergencyStopCallback);

    // Setup services
    ros::ServiceServer modeChangeSrv = n.advertiseService("mode_change", modeChangeCallback);
    ros::ServiceServer emergencyStopSrv = n.advertiseService("trigger_emergency_stop",
                                                             emergencyStopServiceCallback);

    // Create timers
    ros::Timer monitor_timer = n.createTimer(ros::Duration(1.0 / monitor_rate), monitorCallback);
    ros::Timer command_timer = n.createTimer(ros::Duration(1.0 / command_rate), commandCallback);
    ros::Timer diagnostics_timer = n.createTimer(ros::Duration(1.0 / diagnostics_rate), diagnosticsCallback);

    ROS_INFO("All systems initialized. Wheelchair ready.");

    // Use single-threaded spinner (safer for serial communication)
    ros::spin();

    ROS_INFO("Wheelchair node shutting down");
    return 0;
}
