#!/usr/bin/env python
"""
   twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack

   REFACTORED VERSION with smooth velocity mapping and configurable parameters

    Copyright (C) 2012 Jon Stephan.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import rospy
import roslib
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from wheelchair_navigation.msg import SpeedReference
import math


class TwistToMotors():

    def __init__(self):
        rospy.init_node("twist_to_motors")
        nodename = rospy.get_name()
        rospy.loginfo("%s started (refactored version)" % nodename)

        # Load parameters from config file or use defaults
        self.wheel_diameter = rospy.get_param("~wheel_diameter", 0.380)
        self.gear_ratio = rospy.get_param("~gear_ratio", 32)
        self.wheel_base = rospy.get_param("~base_width", 0.59)

        self.max_rpm = rospy.get_param("~max_rpm", 1500)
        self.max_command = rospy.get_param("~max_command", 1000)

        self.min_effective_command = rospy.get_param("~min_effective_command", 15)
        self.min_motor_output = rospy.get_param("~min_motor_output", 100)

        self.enable_ramping = rospy.get_param("~enable_ramping", True)
        self.max_acceleration = rospy.get_param("~max_acceleration", 500)  # cmd/s

        rospy.loginfo("Motor parameters loaded:")
        rospy.loginfo("  Wheel diameter: %.3f m", self.wheel_diameter)
        rospy.loginfo("  Gear ratio: %d", self.gear_ratio)
        rospy.loginfo("  Max RPM: %d", self.max_rpm)
        rospy.loginfo("  Dead-zone: [%d, %d]", self.min_effective_command, self.min_motor_output)
        rospy.loginfo("  Ramping enabled: %s", self.enable_ramping)

        # Publisher
        self.publisher = rospy.Publisher('/speed_ref', SpeedReference, queue_size=10)

        # Subscriber
        rospy.Subscriber('/cmd_vel', Twist, self.twistCallback)

        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)

        # Current and target velocities
        self.target_left = 0.0
        self.target_right = 0.0

        # Previous commands for ramping
        self.prev_cmd_left = 0.0
        self.prev_cmd_right = 0.0

        # Twist message data
        self.dx = 0.0
        self.dr = 0.0
        self.dy = 0.0

        # Timing
        self.last_time = rospy.Time.now()

    def smooth_speed_mapping(self, raw_value):
        """
        Apply smooth speed mapping with dead-zone handling.

        Instead of jumping from 15 to 100, this function:
        - Maps [0, min_effective_command) -> 0
        - Maps [min_effective_command, max_command] -> [min_motor_output, max_command] linearly

        Args:
            raw_value: Raw command value (can be negative)

        Returns:
            Smoothly mapped command value
        """
        # Handle negative values (backward motion disabled for wheelchair)
        if raw_value < 0:
            return 0

        # Dead zone: below threshold means stop
        if raw_value < self.min_effective_command:
            return 0

        # Saturation: cap at maximum
        if raw_value > self.max_command:
            raw_value = self.max_command

        # Linear scaling from [min_effective_command, max_command] to [min_motor_output, max_command]
        # Formula: output = min_output + (input - min_input) * (max_output - min_output) / (max_input - min_input)
        scaled_value = self.min_motor_output + \
                      (raw_value - self.min_effective_command) * \
                      (self.max_command - self.min_motor_output) / \
                      (self.max_command - self.min_effective_command)

        return scaled_value

    def apply_velocity_ramping(self, target_cmd, prev_cmd, dt):
        """
        Apply velocity ramping to prevent sudden acceleration/deceleration.

        Args:
            target_cmd: Target command value
            prev_cmd: Previous command value
            dt: Time delta in seconds

        Returns:
            Ramped command value
        """
        if not self.enable_ramping or dt <= 0:
            return target_cmd

        # Maximum allowed change in this time step
        max_change = self.max_acceleration * dt

        # Calculate desired change
        desired_change = target_cmd - prev_cmd

        # Limit the change
        if abs(desired_change) > max_change:
            if desired_change > 0:
                return prev_cmd + max_change
            else:
                return prev_cmd - max_change

        return target_cmd

    def spin(self):
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(50)
        self.ticks_since_target = self.timeout_ticks

        ###### main loop  ######
        while not rospy.is_shutdown():

            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()


    def spinOnce(self):
        """
        Main control loop iteration.
        Converts twist commands to motor speeds with smooth mapping and ramping.
        """
        # Calculate wheel velocities from twist (differential drive kinematics)
        # v_right = v_linear + omega * wheel_base/2
        # v_left = v_linear - omega * wheel_base/2
        v_right = self.dx + self.dr * self.wheel_base / 2.0  # [m/s]
        v_left = self.dx - self.dr * self.wheel_base / 2.0   # [m/s]

        # Convert linear velocity [m/s] to RPM
        # RPM = (v * gear_ratio * 60) / (wheel_diameter * pi)
        rpm_left = (v_left * self.gear_ratio * 60.0) / (self.wheel_diameter * math.pi)
        rpm_right = (v_right * self.gear_ratio * 60.0) / (self.wheel_diameter * math.pi)

        # Convert RPM to command value [0, 1000]
        # Command = RPM * (max_command / max_rpm)
        cmd_left = rpm_left * (self.max_command / float(self.max_rpm))
        cmd_right = rpm_right * (self.max_command / float(self.max_rpm))

        # Apply smooth speed mapping (handles dead-zone smoothly)
        cmd_left = self.smooth_speed_mapping(cmd_left)
        cmd_right = self.smooth_speed_mapping(cmd_right)

        # Apply velocity ramping if enabled
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        cmd_left = self.apply_velocity_ramping(cmd_left, self.prev_cmd_left, dt)
        cmd_right = self.apply_velocity_ramping(cmd_right, self.prev_cmd_right, dt)

        # Store for next iteration
        self.prev_cmd_left = cmd_left
        self.prev_cmd_right = cmd_right

        # Convert to integer and publish
        speeds = SpeedReference()
        speeds.left = int(round(cmd_left))
        speeds.right = int(round(cmd_right))

        try:
            self.publisher.publish(speeds)
        except Exception as e:
            rospy.logerr("Failed to publish speed reference: %s", str(e))


    def twistCallback(self, msg):
        """
        Callback for incoming twist messages from navigation stack.
        """
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y


if __name__ == '__main__':
    try:
        twistToMotors = TwistToMotors()
        twistToMotors.spin()
    except rospy.ROSInterruptException:
        pass
