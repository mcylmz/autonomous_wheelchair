#ifndef ERROR_RECOVERY_H
#define ERROR_RECOVERY_H

#include <ros/ros.h>
#include <string>
#include <map>
#include "wheelchair_navigation/ChairController.h"

/**
 * @brief Error recovery strategy enumeration
 */
enum class RecoveryStrategy {
    NONE,                    // No recovery action
    RETRY,                   // Retry the operation
    RECONNECT,               // Attempt to reconnect device
    RESET_CONTROLLER,        // Reset controller to safe state
    EMERGENCY_STOP,          // Trigger emergency stop
    NOTIFY_OPERATOR          // Send notification and wait
};

/**
 * @brief Error recovery manager
 *
 * Implements intelligent error recovery strategies with retry logic,
 * exponential backoff, and escalation policies.
 */
class ErrorRecovery {
public:
    /**
     * @brief Construct error recovery manager
     * @param nh ROS node handle
     */
    explicit ErrorRecovery(ros::NodeHandle& nh);

    /**
     * @brief Handle an error with appropriate recovery strategy
     * @param error The error code
     * @param context Additional context about the error
     * @return True if recovery was successful
     */
    bool handleError(WheelchairError error, const std::string& context = "");

    /**
     * @brief Reset error statistics
     */
    void resetStatistics();

    /**
     * @brief Get error count for specific error type
     * @param error Error type
     * @return Number of occurrences
     */
    int getErrorCount(WheelchairError error) const;

    /**
     * @brief Get total error count
     * @return Total errors handled
     */
    int getTotalErrorCount() const;

    /**
     * @brief Check if system is in degraded state
     * @return True if too many errors have occurred
     */
    bool isDegraded() const;

private:
    ros::NodeHandle nh_;
    ros::Publisher errorLogPub_;

    // Configuration
    int maxRetries_;
    double retryBaseDelay_;
    double retryMaxDelay_;
    int degradedThreshold_;
    bool autoRecover_;

    // Error tracking
    std::map<WheelchairError, int> errorCounts_;
    std::map<WheelchairError, ros::Time> lastErrorTime_;
    std::map<WheelchairError, int> consecutiveErrors_;

    // Recovery history
    struct RecoveryAttempt {
        WheelchairError error;
        RecoveryStrategy strategy;
        bool success;
        ros::Time timestamp;
        std::string context;
    };
    std::vector<RecoveryAttempt> recoveryHistory_;

    /**
     * @brief Determine appropriate recovery strategy for error
     * @param error Error code
     * @return Recovery strategy to use
     */
    RecoveryStrategy selectStrategy(WheelchairError error);

    /**
     * @brief Execute a recovery strategy
     * @param strategy Strategy to execute
     * @param error Error being recovered from
     * @param context Error context
     * @return True if recovery successful
     */
    bool executeStrategy(RecoveryStrategy strategy,
                        WheelchairError error,
                        const std::string& context);

    /**
     * @brief Calculate retry delay using exponential backoff
     * @param attemptNumber Attempt number (0-indexed)
     * @return Delay in seconds
     */
    double calculateRetryDelay(int attemptNumber);

    /**
     * @brief Log error for diagnostics
     * @param error Error code
     * @param strategy Recovery strategy used
     * @param success Whether recovery succeeded
     * @param context Error context
     */
    void logError(WheelchairError error,
                  RecoveryStrategy strategy,
                  bool success,
                  const std::string& context);

    /**
     * @brief Load recovery parameters
     */
    void loadParameters();
};

#endif // ERROR_RECOVERY_H
