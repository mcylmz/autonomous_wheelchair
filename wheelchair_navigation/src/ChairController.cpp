#include "wheelchair_navigation/ChairController.h"
#include "wheelchair_navigation/ErrorCodes.h"
#include "wheelchair_navigation/Constants.h"
#include "wheelchair_navigation/jute.h"
#include <ros/ros.h>
#include <chrono>
#include <thread>

ChairController::ChairController(const ros::NodeHandle& nh)
    : device_(nullptr)
    , refReady_(false)
    , controlMode_(-1)
    , pidConfig_(-1)
    , state_(ControlState::DISCONNECTED)
    , emergencyStopActive_(false)
{
    // Load parameters with defaults
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("device_path", devicePath_, "/dev/ttyACM0");
    private_nh.param("command_timeout", commandTimeout_, 0.1);
    private_nh.param("max_retries", maxRetries_, 5);
    private_nh.param("retry_delay", retryDelay_, 1.0);

    ROS_INFO("ChairController initialized with device_path: %s", devicePath_.c_str());
    ROS_INFO("Command timeout: %.2f s, Max retries: %d", commandTimeout_, maxRetries_);

    // Create device instance
    device_ = std::make_unique<RoboteqExtended>();
}

ChairController::~ChairController() {
    // RAII cleanup - stop motors before disconnecting
    ROS_INFO("ChairController shutting down...");

    std::lock_guard<std::mutex> deviceLock(deviceMutex_);
    if (device_ && state_ != ControlState::DISCONNECTED) {
        stopMotors();
    }
    // device_ will be automatically destroyed
}

WheelchairError ChairController::connect() {
    std::lock_guard<std::mutex> stateLock(stateMutex_);
    std::lock_guard<std::mutex> deviceLock(deviceMutex_);

    if (state_ != ControlState::DISCONNECTED) {
        ROS_WARN("Already connected or in invalid state");
        return WheelchairError::INVALID_STATE_TRANSITION;
    }

    state_ = ControlState::INITIALIZING;

    // Retry connection with exponential backoff
    for (int attempt = 0; attempt < maxRetries_; ++attempt) {
        ROS_INFO("Attempting to connect to device at %s (attempt %d/%d)...",
                 devicePath_.c_str(), attempt + 1, maxRetries_);

        int status = device_->Connect(devicePath_);

        if (status == RQ_SUCCESS) {
            ROS_INFO("Successfully connected to Roboteq device");

            // Start script execution
            status = device_->FastSetCommand(_R, 1);
            if (status != RQ_SUCCESS) {
                ROS_ERROR("Failed to start script execution: %d", status);
                state_ = ControlState::FAULT;
                return WheelchairError::DEVICE_COMMUNICATION_ERROR;
            }

            state_ = ControlState::READY;
            return WheelchairError::SUCCESS;
        }

        ROS_WARN("Connection failed with status %d", status);

        if (attempt < maxRetries_ - 1) {
            // Exponential backoff: 1s, 2s, 4s, 8s, 16s
            double delay = retryDelay_ * (1 << attempt);
            ROS_INFO("Retrying in %.1f seconds...", delay);
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay * 1000)));
        }
    }

    ROS_ERROR("Failed to connect after %d attempts", maxRetries_);
    state_ = ControlState::FAULT;
    return WheelchairError::DEVICE_CONNECTION_FAILED;
}

WheelchairError ChairController::setReference(
    const wheelchair_navigation::MotorReference& ref, bool isSpeedMode) {

    std::lock_guard<std::mutex> stateLock(stateMutex_);

    // Check emergency stop
    if (emergencyStopActive_.load()) {
        ROS_WARN_THROTTLE(1.0, "Cannot set reference: emergency stop is active");
        return WheelchairError::EMERGENCY_STOP_ACTIVE;
    }

    // Check state
    if (state_ != ControlState::READY) {
        ROS_ERROR_THROTTLE(1.0, "Cannot set reference: system not ready (state: %d)",
                          static_cast<int>(state_));
        return WheelchairError::DEVICE_CONNECTION_FAILED;
    }

    // Update last reference
    lastRef_ = ref;
    refReady_ = true;

    // Set control mode
    int targetMode = isSpeedMode ? ControlMode::SPEED : ControlMode::TORQUE;
    if (controlMode_ != targetMode) {
        WheelchairError error = stopMotors();
        if (error != WheelchairError::SUCCESS) {
            return error;
        }

        error = setControlMode(targetMode);
        if (error != WheelchairError::SUCCESS) {
            return error;
        }
    }

    // Configure PID if needed
    if (pidConfig_ != targetMode) {
        WheelchairError error = configurePID(targetMode);
        if (error != WheelchairError::SUCCESS) {
            return error;
        }
    }

    return WheelchairError::SUCCESS;
}

WheelchairError ChairController::triggerEmergencyStop(const std::string& reason) {
    std::lock_guard<std::mutex> stateLock(stateMutex_);

    ROS_ERROR("EMERGENCY STOP TRIGGERED: %s", reason.c_str());

    emergencyStopReason_ = reason;
    emergencyStopActive_.store(true);

    ControlState previousState = state_;
    state_ = ControlState::EMERGENCY_STOP;

    // Stop motors immediately
    WheelchairError error = stopMotors();
    if (error != WheelchairError::SUCCESS) {
        ROS_ERROR("Failed to stop motors during emergency stop!");
        state_ = ControlState::FAULT;
        return error;
    }

    // Switch to PWM mode (power stages off)
    error = setControlMode(ControlMode::PWM);
    if (error != WheelchairError::SUCCESS) {
        ROS_ERROR("Failed to switch to PWM mode during emergency stop!");
        state_ = ControlState::FAULT;
        return error;
    }

    ROS_WARN("Emergency stop complete. System in safe state.");
    return WheelchairError::SUCCESS;
}

WheelchairError ChairController::releaseEmergencyStop() {
    std::lock_guard<std::mutex> stateLock(stateMutex_);

    if (!emergencyStopActive_.load()) {
        ROS_WARN("Emergency stop is not active");
        return WheelchairError::SUCCESS;
    }

    if (state_ != ControlState::EMERGENCY_STOP) {
        ROS_ERROR("Cannot release emergency stop: system in invalid state");
        return WheelchairError::INVALID_STATE_TRANSITION;
    }

    ROS_INFO("Releasing emergency stop. Reason was: %s", emergencyStopReason_.c_str());

    emergencyStopActive_.store(false);
    emergencyStopReason_.clear();
    state_ = ControlState::READY;
    refReady_ = false;

    ROS_INFO("Emergency stop released. System ready.");
    return WheelchairError::SUCCESS;
}

WheelchairError ChairController::getMonitorData(
    wheelchair_navigation::MotorMonitor& monitor) const {

    std::lock_guard<std::mutex> deviceLock(deviceMutex_);

    if (!device_ || state_ == ControlState::DISCONNECTED) {
        return WheelchairError::DEVICE_CONNECTION_FAILED;
    }

    std::string telemetry;
    int status = device_->GetTelemetry(telemetry);

    if (status != RQ_SUCCESS) {
        ROS_WARN_THROTTLE(1.0, "GetTelemetry: Error %d", status);
        return WheelchairError::DEVICE_COMMUNICATION_ERROR;
    }

    // Parse telemetry (using jute library as in original code)
    // Note: This requires including jute.h
    try {
        jute::jValue telem_json = jute::parser::parse(telemetry);

        monitor.encoder1 = telem_json["C"][0].as_int();
        monitor.encoder2 = telem_json["C"][1].as_int();
        monitor.rpm1 = telem_json["S"][0].as_int();
        monitor.rpm2 = telem_json["S"][1].as_int();
        monitor.cmd1 = telem_json["M"][0].as_int();
        monitor.cmd2 = telem_json["M"][1].as_int();
        monitor.amps1 = float(telem_json["A"][0].as_int()) / 10.0;
        monitor.amps2 = float(telem_json["A"][1].as_int()) / 10.0;
        monitor.v_int = float(telem_json["V"][0].as_int()) / 10.0;
        monitor.v_batt = float(telem_json["V"][1].as_int()) / 10.0;
        monitor.temp1 = telem_json["T"][0].as_int();
        monitor.temp2 = telem_json["T"][1].as_int();
        monitor.faultFlags = telem_json["FF"].as_int();

        // Set mode string
        std::lock_guard<std::mutex> stateLock(stateMutex_);
        switch (controlMode_) {
            case ControlMode::PWM:
                monitor.mode = "PWM";
                break;
            case ControlMode::SPEED:
                monitor.mode = "Speed";
                break;
            case ControlMode::TORQUE:
                monitor.mode = "Torque";
                break;
            default:
                monitor.mode = "";
                break;
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to parse telemetry JSON: %s", e.what());
        return WheelchairError::DEVICE_COMMUNICATION_ERROR;
    }

    return WheelchairError::SUCCESS;
}

bool ChairController::isEmergencyStopActive() const {
    return emergencyStopActive_.load();
}

ControlState ChairController::getState() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return state_;
}

std::string ChairController::errorToString(WheelchairError error) {
    switch (error) {
        case WheelchairError::SUCCESS:
            return "Success";
        case WheelchairError::DEVICE_CONNECTION_FAILED:
            return "Device connection failed";
        case WheelchairError::DEVICE_COMMUNICATION_ERROR:
            return "Device communication error";
        case WheelchairError::MOTOR_FAULT:
            return "Motor fault detected";
        case WheelchairError::EMERGENCY_STOP_ACTIVE:
            return "Emergency stop is active";
        case WheelchairError::COMMAND_TIMEOUT:
            return "Command timeout";
        case WheelchairError::INVALID_STATE_TRANSITION:
            return "Invalid state transition";
        default:
            return "Unknown error";
    }
}

WheelchairError ChairController::handleCommandTimeout() {
    std::lock_guard<std::mutex> stateLock(stateMutex_);

    if (emergencyStopActive_.load() || state_ != ControlState::READY) {
        return WheelchairError::SUCCESS; // Already in safe state
    }

    ROS_WARN_THROTTLE(5.0, "Command timeout detected - stopping motors");

    // Stop motors
    WheelchairError error = stopMotors();
    if (error != WheelchairError::SUCCESS) {
        return error;
    }

    // Switch to PWM mode (power stages off)
    if (controlMode_ != ControlMode::PWM) {
        error = setControlMode(ControlMode::PWM);
        if (error != WheelchairError::SUCCESS) {
            return error;
        }
    }

    return WheelchairError::SUCCESS;
}

// Private helper methods

WheelchairError ChairController::setControlMode(int mode) {
    // Assumes stateMutex_ is already locked by caller

    std::lock_guard<std::mutex> deviceLock(deviceMutex_);

    int status = device_->FastSetConfig(_MMOD, 1, mode);
    if (status != RQ_SUCCESS) {
        ROS_ERROR("Failed to set control mode for motor 1: %d", status);
        return WheelchairError::DEVICE_COMMUNICATION_ERROR;
    }

    status = device_->FastSetConfig(_MMOD, 2, mode);
    if (status != RQ_SUCCESS) {
        ROS_ERROR("Failed to set control mode for motor 2: %d", status);
        return WheelchairError::DEVICE_COMMUNICATION_ERROR;
    }

    controlMode_ = mode;
    ROS_DEBUG("Control mode set to %d", mode);
    return WheelchairError::SUCCESS;
}

WheelchairError ChairController::configurePID(int mode) {
    // Assumes stateMutex_ is already locked by caller

    std::lock_guard<std::mutex> deviceLock(deviceMutex_);

    int kp, ki, kd;

    if (mode == ControlMode::SPEED) {
        kp = 11;
        ki = 3;
        kd = 0;
    } else if (mode == ControlMode::TORQUE) {
        kp = 0;
        ki = 1;
        kd = 0;
    } else {
        ROS_WARN("PID configuration not needed for mode %d", mode);
        pidConfig_ = mode;
        return WheelchairError::SUCCESS;
    }

    // Set PID gains for both motors
    WheelchairError error = dualSetConfig(_KP, kp);
    if (error != WheelchairError::SUCCESS) return error;

    error = dualSetConfig(_KI, ki);
    if (error != WheelchairError::SUCCESS) return error;

    error = dualSetConfig(_KD, kd);
    if (error != WheelchairError::SUCCESS) return error;

    pidConfig_ = mode;
    ROS_DEBUG("PID configured for mode %d (KP=%d, KI=%d, KD=%d)", mode, kp, ki, kd);
    return WheelchairError::SUCCESS;
}

WheelchairError ChairController::stopMotors() {
    // Assumes stateMutex_ is already locked by caller

    return dualSetCommand(_G, 0);
}

WheelchairError ChairController::dualSetCommand(int command, int value) {
    // Assumes stateMutex_ is already locked by caller

    std::lock_guard<std::mutex> deviceLock(deviceMutex_);

    int status1 = device_->FastSetCommand(command, 1, value);
    if (status1 != RQ_SUCCESS) {
        ROS_ERROR("FastSetCommand failed for motor 1: %d", status1);
        return WheelchairError::DEVICE_COMMUNICATION_ERROR;
    }

    int status2 = device_->FastSetCommand(command, 2, value);
    if (status2 != RQ_SUCCESS) {
        ROS_ERROR("FastSetCommand failed for motor 2: %d", status2);
        return WheelchairError::DEVICE_COMMUNICATION_ERROR;
    }

    return WheelchairError::SUCCESS;
}

WheelchairError ChairController::dualSetConfig(int config, int value) {
    // Assumes stateMutex_ is already locked by caller

    std::lock_guard<std::mutex> deviceLock(deviceMutex_);

    int status1 = device_->FastSetConfig(config, 1, value);
    if (status1 != RQ_SUCCESS) {
        ROS_ERROR("FastSetConfig failed for motor 1: %d", status1);
        return WheelchairError::DEVICE_COMMUNICATION_ERROR;
    }

    int status2 = device_->FastSetConfig(config, 2, value);
    if (status2 != RQ_SUCCESS) {
        ROS_ERROR("FastSetConfig failed for motor 2: %d", status2);
        return WheelchairError::DEVICE_COMMUNICATION_ERROR;
    }

    return WheelchairError::SUCCESS;
}

WheelchairError ChairController::singleSetCommand(int command, int index, int value) {
    // Assumes stateMutex_ is already locked by caller

    std::lock_guard<std::mutex> deviceLock(deviceMutex_);

    int status = device_->FastSetCommand(command, index, value);
    if (status != RQ_SUCCESS) {
        ROS_ERROR("FastSetCommand failed for motor %d: %d", index, status);
        return WheelchairError::DEVICE_COMMUNICATION_ERROR;
    }

    return WheelchairError::SUCCESS;
}
