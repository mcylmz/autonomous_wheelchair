#include "wheelchair_navigation/ErrorRecovery.h"
#include <std_msgs/String.h>
#include <algorithm>
#include <cmath>

ErrorRecovery::ErrorRecovery(ros::NodeHandle& nh)
    : nh_(nh)
{
    loadParameters();

    // Create publisher for error logs
    errorLogPub_ = nh_.advertise<std_msgs::String>("/wheelchair/error_log", 10);

    ROS_INFO("Error recovery system initialized");
    ROS_INFO("  Auto-recover: %s", autoRecover_ ? "enabled" : "disabled");
    ROS_INFO("  Max retries: %d", maxRetries_);
    ROS_INFO("  Retry delays: %.1fs to %.1fs", retryBaseDelay_, retryMaxDelay_);
}

void ErrorRecovery::loadParameters() {
    ros::NodeHandle private_nh("~");

    // Recovery parameters
    private_nh.param("error_recovery/max_retries", maxRetries_, 3);
    private_nh.param("error_recovery/retry_base_delay", retryBaseDelay_, 0.5);
    private_nh.param("error_recovery/retry_max_delay", retryMaxDelay_, 5.0);
    private_nh.param("error_recovery/degraded_threshold", degradedThreshold_, 10);
    private_nh.param("error_recovery/auto_recover", autoRecover_, true);

    // Validate parameters
    if (maxRetries_ < 0) maxRetries_ = 0;
    if (retryBaseDelay_ < 0.1) retryBaseDelay_ = 0.1;
    if (retryMaxDelay_ < retryBaseDelay_) retryMaxDelay_ = retryBaseDelay_ * 10;
    if (degradedThreshold_ < 1) degradedThreshold_ = 10;
}

bool ErrorRecovery::handleError(WheelchairError error, const std::string& context) {
    // Ignore success
    if (error == WheelchairError::SUCCESS) {
        return true;
    }

    // Update error statistics
    errorCounts_[error]++;
    lastErrorTime_[error] = ros::Time::now();
    consecutiveErrors_[error]++;

    ROS_ERROR("Error occurred: %s (context: %s)",
             ChairController::errorToString(error).c_str(),
             context.empty() ? "none" : context.c_str());

    // Check if auto-recovery is enabled
    if (!autoRecover_) {
        ROS_WARN("Auto-recovery disabled, manual intervention required");
        logError(error, RecoveryStrategy::NOTIFY_OPERATOR, false, context);
        return false;
    }

    // Select recovery strategy
    RecoveryStrategy strategy = selectStrategy(error);

    ROS_INFO("Attempting recovery using strategy: %d", static_cast<int>(strategy));

    // Execute recovery with retries
    bool recovered = false;
    for (int attempt = 0; attempt < maxRetries_ && !recovered; ++attempt) {
        if (attempt > 0) {
            double delay = calculateRetryDelay(attempt - 1);
            ROS_INFO("Retry attempt %d/%d after %.2fs delay",
                    attempt + 1, maxRetries_, delay);
            ros::Duration(delay).sleep();
        }

        recovered = executeStrategy(strategy, error, context);
    }

    // Log recovery attempt
    logError(error, strategy, recovered, context);

    // Record in history
    RecoveryAttempt attempt;
    attempt.error = error;
    attempt.strategy = strategy;
    attempt.success = recovered;
    attempt.timestamp = ros::Time::now();
    attempt.context = context;
    recoveryHistory_.push_back(attempt);

    // Limit history size
    if (recoveryHistory_.size() > 100) {
        recoveryHistory_.erase(recoveryHistory_.begin());
    }

    // Reset consecutive errors on success
    if (recovered) {
        consecutiveErrors_[error] = 0;
        ROS_INFO("Recovery successful");
    } else {
        ROS_ERROR("Recovery failed after %d attempts", maxRetries_);
    }

    return recovered;
}

RecoveryStrategy ErrorRecovery::selectStrategy(WheelchairError error) {
    // Check for escalation based on consecutive errors
    int consecutive = consecutiveErrors_[error];

    switch (error) {
        case WheelchairError::DEVICE_COMMUNICATION_ERROR:
            if (consecutive > 5) {
                return RecoveryStrategy::RECONNECT;
            }
            return RecoveryStrategy::RETRY;

        case WheelchairError::DEVICE_CONNECTION_FAILED:
            return RecoveryStrategy::RECONNECT;

        case WheelchairError::MOTOR_FAULT:
            if (consecutive > 2) {
                return RecoveryStrategy::EMERGENCY_STOP;
            }
            return RecoveryStrategy::RESET_CONTROLLER;

        case WheelchairError::EMERGENCY_STOP_ACTIVE:
            // Don't auto-recover from emergency stop
            return RecoveryStrategy::NOTIFY_OPERATOR;

        case WheelchairError::COMMAND_TIMEOUT:
            return RecoveryStrategy::RESET_CONTROLLER;

        case WheelchairError::INVALID_STATE_TRANSITION:
            return RecoveryStrategy::RESET_CONTROLLER;

        default:
            return RecoveryStrategy::RETRY;
    }
}

bool ErrorRecovery::executeStrategy(RecoveryStrategy strategy,
                                    WheelchairError error,
                                    const std::string& context) {
    switch (strategy) {
        case RecoveryStrategy::NONE:
            return false;

        case RecoveryStrategy::RETRY:
            // Simply return false to trigger retry at higher level
            ROS_INFO("Using RETRY strategy - operation will be retried");
            return false;

        case RecoveryStrategy::RECONNECT:
            ROS_WARN("RECONNECT strategy: Device reconnection needed");
            ROS_WARN("This requires controller re-initialization - not auto-recoverable");
            return false;

        case RecoveryStrategy::RESET_CONTROLLER:
            ROS_WARN("RESET_CONTROLLER strategy: Attempting to reset to safe state");
            // This would need controller reference - simplified here
            ROS_WARN("Controller reset would be performed here");
            return false;

        case RecoveryStrategy::EMERGENCY_STOP:
            ROS_ERROR("EMERGENCY_STOP strategy: Triggering emergency stop");
            // This would need controller reference - simplified here
            ROS_ERROR("Emergency stop would be triggered here");
            return false;

        case RecoveryStrategy::NOTIFY_OPERATOR:
            ROS_ERROR("NOTIFY_OPERATOR strategy: Manual intervention required");
            ROS_ERROR("Error: %s, Context: %s",
                     ChairController::errorToString(error).c_str(),
                     context.c_str());
            return false;

        default:
            return false;
    }
}

double ErrorRecovery::calculateRetryDelay(int attemptNumber) {
    // Exponential backoff: base * 2^attempt, capped at max
    double delay = retryBaseDelay_ * std::pow(2.0, attemptNumber);
    return std::min(delay, retryMaxDelay_);
}

void ErrorRecovery::logError(WheelchairError error,
                             RecoveryStrategy strategy,
                             bool success,
                             const std::string& context) {
    // Create log message
    std_msgs::String msg;
    std::stringstream ss;

    ss << ros::Time::now() << " | "
       << "ERROR: " << ChairController::errorToString(error) << " | "
       << "STRATEGY: " << static_cast<int>(strategy) << " | "
       << "RESULT: " << (success ? "SUCCESS" : "FAILED") << " | "
       << "CONTEXT: " << context;

    msg.data = ss.str();
    errorLogPub_.publish(msg);

    // Also log to ROS console
    if (success) {
        ROS_INFO("Error recovered: %s", msg.data.c_str());
    } else {
        ROS_ERROR("Recovery failed: %s", msg.data.c_str());
    }
}

void ErrorRecovery::resetStatistics() {
    errorCounts_.clear();
    lastErrorTime_.clear();
    consecutiveErrors_.clear();
    recoveryHistory_.clear();
    ROS_INFO("Error statistics reset");
}

int ErrorRecovery::getErrorCount(WheelchairError error) const {
    auto it = errorCounts_.find(error);
    return (it != errorCounts_.end()) ? it->second : 0;
}

int ErrorRecovery::getTotalErrorCount() const {
    int total = 0;
    for (const auto& pair : errorCounts_) {
        total += pair.second;
    }
    return total;
}

bool ErrorRecovery::isDegraded() const {
    return getTotalErrorCount() >= degradedThreshold_;
}
