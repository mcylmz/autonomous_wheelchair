#!/usr/bin/env python
import rospy
import tf2_ros
from sensor_msgs.msg import Imu


def imu_accel_callback(msg):
    rospy.loginfo("I heard an imu accel message!")

def imu_gyro_callback(msg):
    rospy.loginfo("I heard an imu gyro message")


if __name__ == '__main__':
    rospy.init_node('camera_imu_to_baselink')
    rospy.Subscriber('/camera/accel/sample', Imu, imu_accel_callback)
    rospy.Subscriber('/camera/gyro/sample', Imu, imu_gyro_callback)
    
    rospy.spin()

