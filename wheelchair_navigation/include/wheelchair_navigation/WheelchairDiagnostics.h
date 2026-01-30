#ifndef WHEELCHAIR_DIAGNOSTICS_H
#define WHEELCHAIR_DIAGNOSTICS_H

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <wheelchair_navigation/MotorMonitor.h>
#include <wheelchair_navigation/EmergencyStop.h>
#include <std_msgs/String.h>

// Forward declaration
class ChairController;

/**
 * @brief Comprehensive diagnostics monitoring for wheelchair system
 *
 * Monitors and reports:
 * - Motor controller status and faults
 * - Battery voltage with warning/error thresholds
 * - Motor temperatures with safety limits
 * - Emergency stop status
 * - Communication health
 * - Planning success rates
 */
class WheelchairDiagnostics {
public:
    /**
     * @brief Construct diagnostics system
     * @param nh ROS node handle
     */
    explicit WheelchairDiagnostics(ros::NodeHandle& nh);

    /**
     * @brief Update diagnostics (call periodically)
     */
    void update();

    /**
     * @brief Update motor monitor data
     * @param monitor Latest motor telemetry
     */
    void updateMotorData(const wheelchair_navigation::MotorMonitor& monitor);

    /**
     * @brief Update emergency stop status
     * @param estop Emergency stop message
     */
    void updateEmergencyStopStatus(const wheelchair_navigation::EmergencyStop& estop);

    /**
     * @brief Record planning attempt
     * @param success True if planning succeeded
     */
    void recordPlanningAttempt(bool success);

    /**
     * @brief Record navigation goal attempt
     * @param success True if goal was reached
     */
    void recordNavigationGoal(bool success);

private:
    // ROS interface
    ros::NodeHandle nh_;
    diagnostic_updater::Updater updater_;

    // Latest data
    wheelchair_navigation::MotorMonitor latestMotorData_;
    wheelchair_navigation::EmergencyStop latestEstopStatus_;
    ros::Time lastMotorDataTime_;
    ros::Time lastEstopUpdateTime_;

    // Diagnostic thresholds (loaded from parameters)
    double batteryVoltageWarn_;
    double batteryVoltageError_;
    double motorTempWarn_;
    double motorTempError_;
    double motorCurrentWarn_;
    double motorCurrentError_;
    double dataTimeoutThreshold_;

    // Statistics
    int planningAttempts_;
    int planningSuccesses_;
    int navigationAttempts_;
    int navigationSuccesses_;
    ros::Time statisticsResetTime_;

    // Diagnostic task functions
    void checkMotorController(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void checkBatteryStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void checkMotorTemperatures(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void checkMotorCurrents(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void checkEmergencyStop(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void checkCommunication(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void checkPlanningPerformance(diagnostic_updater::DiagnosticStatusWrapper& stat);

    // Helper functions
    std::string faultFlagsToString(int faultFlags) const;
    void loadParameters();
};

#endif // WHEELCHAIR_DIAGNOSTICS_H
