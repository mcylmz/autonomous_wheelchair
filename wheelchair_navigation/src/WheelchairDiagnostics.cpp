#include "wheelchair_navigation/WheelchairDiagnostics.h"
#include <sstream>
#include <iomanip>

WheelchairDiagnostics::WheelchairDiagnostics(ros::NodeHandle& nh)
    : nh_(nh)
    , planningAttempts_(0)
    , planningSuccesses_(0)
    , navigationAttempts_(0)
    , navigationSuccesses_(0)
{
    // Load parameters
    loadParameters();

    // Set hardware ID
    updater_.setHardwareID("autonomous_wheelchair");

    // Add diagnostic tasks
    updater_.add("Motor Controller", this, &WheelchairDiagnostics::checkMotorController);
    updater_.add("Battery Status", this, &WheelchairDiagnostics::checkBatteryStatus);
    updater_.add("Motor Temperatures", this, &WheelchairDiagnostics::checkMotorTemperatures);
    updater_.add("Motor Currents", this, &WheelchairDiagnostics::checkMotorCurrents);
    updater_.add("Emergency Stop", this, &WheelchairDiagnostics::checkEmergencyStop);
    updater_.add("Communication", this, &WheelchairDiagnostics::checkCommunication);
    updater_.add("Planning Performance", this, &WheelchairDiagnostics::checkPlanningPerformance);

    // Initialize timestamps
    lastMotorDataTime_ = ros::Time::now();
    lastEstopUpdateTime_ = ros::Time::now();
    statisticsResetTime_ = ros::Time::now();

    // Initialize emergency stop status
    latestEstopStatus_.active = false;

    ROS_INFO("Wheelchair diagnostics system initialized");
    ROS_INFO("  Battery thresholds: WARN < %.1fV, ERROR < %.1fV",
             batteryVoltageWarn_, batteryVoltageError_);
    ROS_INFO("  Temperature thresholds: WARN > %.0fC, ERROR > %.0fC",
             motorTempWarn_, motorTempError_);
}

void WheelchairDiagnostics::loadParameters() {
    ros::NodeHandle private_nh("~");

    // Battery voltage thresholds
    private_nh.param("diagnostics/battery_voltage_warn", batteryVoltageWarn_, 22.0);
    private_nh.param("diagnostics/battery_voltage_error", batteryVoltageError_, 20.0);

    // Motor temperature thresholds (Celsius)
    private_nh.param("diagnostics/motor_temp_warn", motorTempWarn_, 60.0);
    private_nh.param("diagnostics/motor_temp_error", motorTempError_, 75.0);

    // Motor current thresholds (Amps)
    private_nh.param("diagnostics/motor_current_warn", motorCurrentWarn_, 25.0);
    private_nh.param("diagnostics/motor_current_error", motorCurrentError_, 30.0);

    // Communication timeout
    private_nh.param("diagnostics/data_timeout", dataTimeoutThreshold_, 0.5);

    // Validate parameters
    if (batteryVoltageError_ >= batteryVoltageWarn_) {
        ROS_WARN("Battery error threshold should be < warn threshold, swapping");
        std::swap(batteryVoltageError_, batteryVoltageWarn_);
    }

    if (motorTempError_ <= motorTempWarn_) {
        ROS_WARN("Motor temp error threshold should be > warn threshold, swapping");
        std::swap(motorTempError_, motorTempWarn_);
    }
}

void WheelchairDiagnostics::update() {
    updater_.update();
}

void WheelchairDiagnostics::updateMotorData(const wheelchair_navigation::MotorMonitor& monitor) {
    latestMotorData_ = monitor;
    lastMotorDataTime_ = ros::Time::now();
}

void WheelchairDiagnostics::updateEmergencyStopStatus(const wheelchair_navigation::EmergencyStop& estop) {
    latestEstopStatus_ = estop;
    lastEstopUpdateTime_ = ros::Time::now();
}

void WheelchairDiagnostics::recordPlanningAttempt(bool success) {
    planningAttempts_++;
    if (success) {
        planningSuccesses_++;
    }
}

void WheelchairDiagnostics::recordNavigationGoal(bool success) {
    navigationAttempts_++;
    if (success) {
        navigationSuccesses_++;
    }
}

void WheelchairDiagnostics::checkMotorController(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    // Check for fault flags
    int faultFlags = latestMotorData_.faultFlags;

    if (faultFlags != 0) {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                     "Motor fault detected: 0x%02X (%s)",
                     faultFlags, faultFlagsToString(faultFlags).c_str());
    } else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Motors operating normally");
    }

    // Add detailed information
    stat.add("Fault Flags", faultFlags);
    stat.add("Control Mode", latestMotorData_.mode);
    stat.add("Motor 1 Command", latestMotorData_.cmd1);
    stat.add("Motor 2 Command", latestMotorData_.cmd2);
    stat.add("Motor 1 RPM", latestMotorData_.rpm1);
    stat.add("Motor 2 RPM", latestMotorData_.rpm2);
    stat.add("Encoder 1", latestMotorData_.encoder1);
    stat.add("Encoder 2", latestMotorData_.encoder2);
}

void WheelchairDiagnostics::checkBatteryStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    double voltage = latestMotorData_.v_batt;
    double internalVoltage = latestMotorData_.v_int;

    if (voltage < batteryVoltageError_) {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                     "Critical battery voltage: %.1fV (< %.1fV)",
                     voltage, batteryVoltageError_);
    } else if (voltage < batteryVoltageWarn_) {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
                     "Low battery voltage: %.1fV (< %.1fV)",
                     voltage, batteryVoltageWarn_);
    } else {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
                     "Battery OK: %.1fV", voltage);
    }

    stat.add("Battery Voltage (V)", voltage);
    stat.add("Internal Voltage (V)", internalVoltage);
    stat.add("Voltage Warn Threshold (V)", batteryVoltageWarn_);
    stat.add("Voltage Error Threshold (V)", batteryVoltageError_);

    // Add estimated runtime if we know battery capacity
    // This could be enhanced with battery capacity parameter
}

void WheelchairDiagnostics::checkMotorTemperatures(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    int temp1 = latestMotorData_.temp1;
    int temp2 = latestMotorData_.temp2;
    int maxTemp = std::max(temp1, temp2);

    if (maxTemp > motorTempError_) {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                     "Critical motor temperature: M1=%dC, M2=%dC (> %.0fC)",
                     temp1, temp2, motorTempError_);
    } else if (maxTemp > motorTempWarn_) {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
                     "High motor temperature: M1=%dC, M2=%dC (> %.0fC)",
                     temp1, temp2, motorTempWarn_);
    } else {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
                     "Motor temperatures normal: M1=%dC, M2=%dC", temp1, temp2);
    }

    stat.add("Motor 1 Temperature (C)", temp1);
    stat.add("Motor 2 Temperature (C)", temp2);
    stat.add("Temperature Warn Threshold (C)", motorTempWarn_);
    stat.add("Temperature Error Threshold (C)", motorTempError_);
}

void WheelchairDiagnostics::checkMotorCurrents(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    double current1 = latestMotorData_.amps1;
    double current2 = latestMotorData_.amps2;
    double maxCurrent = std::max(std::abs(current1), std::abs(current2));

    if (maxCurrent > motorCurrentError_) {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                     "Excessive motor current: M1=%.1fA, M2=%.1fA (> %.1fA)",
                     current1, current2, motorCurrentError_);
    } else if (maxCurrent > motorCurrentWarn_) {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
                     "High motor current: M1=%.1fA, M2=%.1fA (> %.1fA)",
                     current1, current2, motorCurrentWarn_);
    } else {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
                     "Motor currents normal: M1=%.1fA, M2=%.1fA", current1, current2);
    }

    stat.add("Motor 1 Current (A)", current1);
    stat.add("Motor 2 Current (A)", current2);
    stat.add("Current Warn Threshold (A)", motorCurrentWarn_);
    stat.add("Current Error Threshold (A)", motorCurrentError_);
}

void WheelchairDiagnostics::checkEmergencyStop(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (latestEstopStatus_.active) {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                     "EMERGENCY STOP ACTIVE: %s",
                     latestEstopStatus_.reason.c_str());
        stat.add("Active", "YES");
        stat.add("Reason", latestEstopStatus_.reason);

        double duration = (ros::Time::now() - latestEstopStatus_.timestamp).toSec();
        stat.add("Duration (s)", duration);
    } else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Emergency stop not active");
        stat.add("Active", "NO");
    }
}

void WheelchairDiagnostics::checkCommunication(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    ros::Time now = ros::Time::now();
    double motorDataAge = (now - lastMotorDataTime_).toSec();

    if (motorDataAge > dataTimeoutThreshold_) {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                     "Motor data stale: %.2fs old (> %.2fs)",
                     motorDataAge, dataTimeoutThreshold_);
    } else {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
                     "Communication healthy: %.3fs latency", motorDataAge);
    }

    stat.add("Motor Data Age (s)", motorDataAge);
    stat.add("Timeout Threshold (s)", dataTimeoutThreshold_);
    stat.add("Last Update", lastMotorDataTime_.toSec());
}

void WheelchairDiagnostics::checkPlanningPerformance(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    double planningSuccessRate = 0.0;
    double navigationSuccessRate = 0.0;

    if (planningAttempts_ > 0) {
        planningSuccessRate = 100.0 * planningSuccesses_ / planningAttempts_;
    }

    if (navigationAttempts_ > 0) {
        navigationSuccessRate = 100.0 * navigationSuccesses_ / navigationAttempts_;
    }

    // Determine status based on success rates
    if (planningAttempts_ > 10 && planningSuccessRate < 50.0) {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                     "Low planning success: %.1f%% (%d/%d)",
                     planningSuccessRate, planningSuccesses_, planningAttempts_);
    } else if (planningAttempts_ > 10 && planningSuccessRate < 80.0) {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
                     "Moderate planning success: %.1f%% (%d/%d)",
                     planningSuccessRate, planningSuccesses_, planningAttempts_);
    } else if (planningAttempts_ > 0) {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
                     "Planning success: %.1f%% (%d/%d)",
                     planningSuccessRate, planningSuccesses_, planningAttempts_);
    } else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No planning attempts yet");
    }

    stat.add("Planning Attempts", planningAttempts_);
    stat.add("Planning Successes", planningSuccesses_);
    stat.addf("Planning Success Rate", "%.1f%%", planningSuccessRate);

    stat.add("Navigation Attempts", navigationAttempts_);
    stat.add("Navigation Successes", navigationSuccesses_);
    stat.addf("Navigation Success Rate", "%.1f%%", navigationSuccessRate);

    double uptime = (ros::Time::now() - statisticsResetTime_).toSec();
    stat.add("Statistics Period (s)", uptime);
}

std::string WheelchairDiagnostics::faultFlagsToString(int faultFlags) const {
    if (faultFlags == 0) {
        return "None";
    }

    std::stringstream ss;
    bool first = true;

    // Common Roboteq fault flags
    if (faultFlags & 0x01) {
        ss << "Overheat";
        first = false;
    }
    if (faultFlags & 0x02) {
        if (!first) ss << ", ";
        ss << "Overvoltage";
        first = false;
    }
    if (faultFlags & 0x04) {
        if (!first) ss << ", ";
        ss << "Undervoltage";
        first = false;
    }
    if (faultFlags & 0x08) {
        if (!first) ss << ", ";
        ss << "Short Circuit";
        first = false;
    }
    if (faultFlags & 0x10) {
        if (!first) ss << ", ";
        ss << "Emergency Stop";
        first = false;
    }
    if (faultFlags & 0x20) {
        if (!first) ss << ", ";
        ss << "MOSFET Failure";
        first = false;
    }
    if (faultFlags & 0x40) {
        if (!first) ss << ", ";
        ss << "Startup Config";
        first = false;
    }

    return ss.str();
}
