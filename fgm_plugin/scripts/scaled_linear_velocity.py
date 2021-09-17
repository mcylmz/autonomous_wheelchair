#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

linear_vel = 0.0
mass_factor = 0.0

def linVelCallback(msg):
    # rospy.loginfo('Linear velocity from Fuzzy1: %s', msg.data)
    global linear_vel
    linear_vel = msg.data


def massScalingCallback(msg):
    # rospy.loginfo('Mass scaling factor from Fuzzy2: %s', msg.data)
    global mass_factor
    mass_factor = msg.data


if __name__ == '__main__':
    
    rospy.init_node('scaled_lin_vel_calc', anonymous=True)

    rospy.Subscriber('/fuzzy_lin_vel', Float32, linVelCallback)
    rospy.Subscriber('/mass_scaling_factor', Float32, massScalingCallback)

    lin_vel_pub = rospy.Publisher('scaled_lin_vel', Float32, queue_size=10)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        output_lin_vel = 2 * linear_vel * mass_factor
        # rospy.loginfo("Output lin vel from scaled_linear_velocity: %f", output_lin_vel)
        if (output_lin_vel > 1.):
            output_lin_vel = 1.0
        lin_vel_pub.publish(output_lin_vel)
        rate.sleep()