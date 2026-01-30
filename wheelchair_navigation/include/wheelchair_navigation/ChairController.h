#ifndef __ChairController_H_
#define __ChairController_H_

#include <memory>
#include <mutex>
#include <atomic>
#include <string>
#include <ros/ros.h>

#include "wheelchair_navigation/RoboteqExtended.h"
#include "wheelchair_navigation/MotorMonitor.h"
#include "wheelchair_navigation/MotorReference.h"
#include "wheelchair_navigation/SpeedReference.h"

// Control modes
namespace ControlMode {
    constexpr int PWM = 0;
    constexpr int SPEED = 1;
    constexpr int TORQUE = 5;
}

// Control state enum for state machine
enum class ControlState {
    DISCONNECTED,
    INITIALIZING,
    READY,
    EMERGENCY_STOP,
    FAULT
};

// Error codes for structured error reporting
enum class WheelchairError {
    SUCCESS = 0,
    DEVICE_CONNECTION_FAILED,
    DEVICE_COMMUNICATION_ERROR,
    MOTOR_FAULT,
    EMERGENCY_STOP_ACTIVE,
    COMMAND_TIMEOUT,
    INVALID_STATE_TRANSITION
};

/**
 * @brief Thread-safe motor controller encapsulating RoboteqExtended device
 *
 * This class uses RAII to manage the device lifetime and mutex guards for thread safety.
 * It implements a state machine for control mode transitions and provides emergency stop
 * functionality.
 */
class ChairController {
public:
    /**
     * @brief Construct a new ChairController
     * @param nh ROS node handle for parameter access
     */
    explicit ChairController(const ros::NodeHandle& nh);

    /**
     * @brief Destroy the ChairController (RAII cleanup)
     */
    ~ChairController();

    // Delete copy constructor and assignment operator (non-copyable due to mutex)
    ChairController(const ChairController&) = delete;
    ChairController& operator=(const ChairController&) = delete;

    /**
     * @brief Connect to the motor controller with retry logic
     * @return WheelchairError error code
     */
    WheelchairError connect();

    /**
     * @brief Set motor reference (speed or torque)
     * @param ref Motor reference message
     * @param isSpeedMode True for speed control, false for torque control
     * @return WheelchairError error code
     */
    WheelchairError setReference(const wheelchair_navigation::MotorReference& ref, bool isSpeedMode);

    /**
     * @brief Trigger emergency stop
     * @param reason Reason for emergency stop
     * @return WheelchairError error code
     */
    WheelchairError triggerEmergencyStop(const std::string& reason);

    /**
     * @brief Release emergency stop
     * @return WheelchairError error code
     */
    WheelchairError releaseEmergencyStop();

    /**
     * @brief Get motor monitor data (telemetry)
     * @param monitor Output parameter for monitor data
     * @return WheelchairError error code
     */
    WheelchairError getMonitorData(wheelchair_navigation::MotorMonitor& monitor) const;

    /**
     * @brief Check if emergency stop is active
     * @return true if emergency stop is active
     */
    bool isEmergencyStopActive() const;

    /**
     * @brief Get current control state
     * @return Current ControlState
     */
    ControlState getState() const;

    /**
     * @brief Get error code as string
     * @param error Error code
     * @return String representation of error
     */
    static std::string errorToString(WheelchairError error);

    /**
     * @brief Handle command timeout (called when no commands received)
     * @return WheelchairError error code
     */
    WheelchairError handleCommandTimeout();

private:
    // RAII device ownership
    std::unique_ptr<RoboteqExtended> device_;

    // Thread safety mutexes
    mutable std::mutex stateMutex_;
    mutable std::mutex deviceMutex_;

    // State variables (protected by stateMutex_)
    wheelchair_navigation::MotorReference lastRef_;
    bool refReady_;
    int controlMode_;
    int pidConfig_;
    ControlState state_;
    std::string emergencyStopReason_;

    // Atomic for lock-free emergency stop check
    std::atomic<bool> emergencyStopActive_;

    // Configuration parameters
    std::string devicePath_;
    double commandTimeout_;
    int maxRetries_;
    double retryDelay_;

    /**
     * @brief Set control mode with state validation
     * @param mode Target control mode (PWM, SPEED, TORQUE)
     * @return WheelchairError error code
     */
    WheelchairError setControlMode(int mode);

    /**
     * @brief Configure PID parameters for control mode
     * @param mode Control mode
     * @return WheelchairError error code
     */
    WheelchairError configurePID(int mode);

    /**
     * @brief Stop motors immediately
     * @return WheelchairError error code
     */
    WheelchairError stopMotors();

    /**
     * @brief Set dual motor command (both motors)
     * @param command Command type
     * @param value Command value
     * @return WheelchairError error code
     */
    WheelchairError dualSetCommand(int command, int value);

    /**
     * @brief Set dual motor configuration (both motors)
     * @param config Configuration parameter
     * @param value Configuration value
     * @return WheelchairError error code
     */
    WheelchairError dualSetConfig(int config, int value);

    /**
     * @brief Set single motor command
     * @param command Command type
     * @param index Motor index (1 or 2)
     * @param value Command value
     * @return WheelchairError error code
     */
    WheelchairError singleSetCommand(int command, int index, int value);
};

#endif // __ChairController_H_
