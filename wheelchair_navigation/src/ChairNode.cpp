#include <iostream>
#include <stdint.h>
#include <cstdlib>

#include <ros/ros.h>
#include <wheelchair_navigation/MotorMonitor.h>
#include <wheelchair_navigation/MotorReference.h>
#include <wheelchair_navigation/SpeedReference.h>
#include <wheelchair_navigation/ModeChange.h>

#include "wheelchair_navigation/ChairNode.h"

#include "wheelchair_navigation/RoboteqExtended.h"
#include "wheelchair_navigation/ErrorCodes.h"
#include "wheelchair_navigation/Constants.h"

#include "wheelchair_navigation/jute.h" // For JSON parsing

// Global Variables
RoboteqExtended device;

wheelchair_navigation::MotorReference last_ref; // Used only once per publish
wheelchair_navigation::SpeedReference last_speed; // Used only once per publish
bool ref_ready = false;
int control_mode = -1;
int pid_config = -1;

// Function Prototypes
// void referenceCallback(const chair::MotorReference::ConstPtr& msg);
// bool modeChangeCallback(chair::ModeChange::Request &req, chair::ModeChange::Response &res);

int singleSetCommand(const int command, const int index, const int arg) {
	int status = device.FastSetCommand(command, index, arg);
	ROS_WARN_COND(status != RQ_SUCCESS, "FastSetCommand: Error %i", status);
	return status;
}

int dualSetConfig(const int config, const int arg) {
	int status_1 = device.FastSetConfig(config, 1, arg);
	ROS_WARN_COND(status_1 != RQ_SUCCESS, "FastSetConfig: Error %i", status_1);
	int status_2 = device.FastSetConfig(config, 2, arg);
	ROS_WARN_COND(status_2 != RQ_SUCCESS, "FastSetConfig: Error %i", status_2);
	if ((status_1 == RQ_SUCCESS) && (status_2 == RQ_SUCCESS))
		return RQ_SUCCESS;
	else
		return RQ_SET_CONFIG_FAILED;
}

int dualSetCommand(const int command, const int arg) {
	int status_1 = device.FastSetCommand(command, 1, arg);
	ROS_WARN_COND(status_1 != RQ_SUCCESS, "FastSetCommand: Error %i", status_1);
	int status_2 = device.FastSetCommand(command, 2, arg);
	ROS_WARN_COND(status_2 != RQ_SUCCESS, "FastSetCommand: Error %i", status_2);
	if ((status_1 == RQ_SUCCESS) && (status_2 == RQ_SUCCESS))
		return RQ_SUCCESS;
	else
		return RQ_SET_COMMAND_FAILED;
}

// Message Callback Functions
void referenceCallback(const wheelchair_navigation::MotorReference::ConstPtr& msg)
{
	last_ref.ref1 = msg->ref1;
	last_ref.ref2 = msg->ref2;

	ref_ready = true;

	if (control_mode != CTRL_TORQUE)
	{
		dualSetCommand(_G, 0); // Stop motors

		int status = dualSetConfig(_MMOD, CTRL_TORQUE);
		if (status == RQ_SUCCESS)
			control_mode = CTRL_TORQUE;
	}

	if (pid_config != CTRL_TORQUE)
	{
		if ((dualSetConfig(_KP, 0) == RQ_SUCCESS) &&
			(dualSetConfig(_KI, 1) == RQ_SUCCESS) &&
			(dualSetConfig(_KD, 0) == RQ_SUCCESS))
		{
			pid_config = CTRL_TORQUE;
		}
	}
}

void speedRefCallback(const wheelchair_navigation::SpeedReference::ConstPtr& msg)
{
	last_ref.ref1 = msg->left;
	last_ref.ref2 = msg->right;

	ref_ready = true;

	// Run once for every serial publish to topic
	if (control_mode != CTRL_SPEED)
	{
		dualSetCommand(_G, 0); // Stop motors

		int status = dualSetConfig(_MMOD, CTRL_SPEED);
		if (status == RQ_SUCCESS)
			control_mode = CTRL_SPEED;
	}

	if (pid_config != CTRL_SPEED)
	{
		if ((dualSetConfig(_KP, 11) == RQ_SUCCESS) &&
			(dualSetConfig(_KI, 03) == RQ_SUCCESS) &&
			(dualSetConfig(_KD, 00) == RQ_SUCCESS))
		{
			pid_config = CTRL_SPEED;
		}
	}
}

// Service Callback Functions
bool modeChangeCallback(wheelchair_navigation::ModeChange::Request &req,
		wheelchair_navigation::ModeChange::Response &res)
{
	// Fisrt, stop the motors
	device.SetCommand(_G, 1, 0);
	device.SetCommand(_G, 2, 0);
	
	// Then perform mode change
	device.SetConfig(_MMOD, 1, req.newMode);
	return true;
}

void monitorCallBack(const ros::TimerEvent &timer_event, ros::Publisher &monitor_publisher)
{
	// ROS_INFO("monitorCallBack triggered");
	wheelchair_navigation::MotorMonitor monitor;

	string telemetry;
	int status = device.GetTelemetry(telemetry);
	ROS_WARN_COND(status != RQ_SUCCESS, "GetTelemetry: Error %i", status);
	
	if (status == RQ_SUCCESS) {
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

		switch (control_mode)
		{
		case CTRL_PWM:
			monitor.mode = "PWM";
			break;
		
		case CTRL_SPEED:
			monitor.mode = "Speed";
			break;

		case CTRL_TORQUE:
			monitor.mode = "Torque";
			break;

		default:
			monitor.mode = "";
			break;
		}

		monitor_publisher.publish(monitor);
	}
}

void commandCallBack(const ros::TimerEvent &timer_event)
{
	// ROS_INFO("commandCallBack triggered");
	static bool motor_stopped = false;
	int status;

	if (ref_ready) {

		singleSetCommand(_G, 1, last_ref.ref1);
		singleSetCommand(_G, 2, last_ref.ref2);

		// If it stays false until the next call, motors will be stopped
		ref_ready = false;

		if (motor_stopped) motor_stopped = false;

	} else { 
		if (!motor_stopped) {

			motor_stopped = true;
		}
		// First, stop the motors
		dualSetCommand(_G, 0);
		
		// // Then put driver into PWM mode so that power stages will be turned off
		if (control_mode != CTRL_PWM) {
			status = dualSetConfig(_MMOD, CTRL_PWM);
			if (status == RQ_SUCCESS)
				control_mode = CTRL_PWM;
		}
	}
}


int main(int argc, char* argv[])
{
	// Initializing roboteq interface
	int status = device.Connect("/dev/ttyACM0");
	if(status != RQ_SUCCESS)
	{
		cout << "Error connecting to device: " << status << "." << endl;
		return 1;
	}
	cout << "Roboteq Interface initialized.\n";

	// Start script execution
	device.FastSetCommand(_R, 1);
	
	// Initializing ROS interface
	ros::init(argc, argv, "chair");
	ros::NodeHandle n;
	ros::Publisher monitorPub = n.advertise<wheelchair_navigation::MotorMonitor>("motor_monitor", 1000);
	ros::Subscriber referenceSub = n.subscribe("motor_ref", 1000, referenceCallback);
	ros::Subscriber speedRef = n.subscribe("speed_ref", 1000, speedRefCallback);
	ros::ServiceServer modeChangeSrv = n.advertiseService("mode_change", modeChangeCallback);

	wheelchair_navigation::MotorMonitor monitor;
	
	// It is not possible to use serial port in multiple threads as concurrently
	ros::MultiThreadedSpinner spinner(1);

	ros::Timer monitor_timer = n.createTimer(ros::Duration(0.02), boost::bind(monitorCallBack, _1, monitorPub));
	ros::Timer command_timer = n.createTimer(ros::Duration(0.1), commandCallBack);

	spinner.spin();
	// ros::spin();

	return 0;
}

