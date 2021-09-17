#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
#import matplotlib.pyplot as plt
import numpy as np

#distance of the obstacle at exactly x degree.: 1.57079994678
#rosmsg show sensor_msgs/LaserScan 
#std_msgs/Header header
#uint32 seq
# #time stamp
#string frame_id
#float32 angle_min
#float32 angle_max
#float32 angle_increment
#float32 time_increment
#float32 scan_time
#float32 range_min
#float32 range_max
#float32[] ranges
#float32[] intensities

def callback(msg):


    # msg.intensities = msg.intensities[lower_band:upper_band]
    # msg.intensities = msg.intensities[::-1]
    
    LAD = 3.0
    msg.range_max = LAD - 0.1
    msg.ranges = list(msg.ranges)
    # msg.intensities = list(msg.intensities)
    for i in range(0, len(msg.ranges)):
        if msg.ranges[i] > LAD: #and (not np.isinf(msg.ranges[i]))
            msg.ranges[i] = 99.0
            # msg.intensities[i] = 99
    # rospy.loginfo("intensities %s",z2)

    msg.ranges = tuple(msg.ranges)
    # msg.intensities = tuple(msg.intensities)
    pub.publish(msg)
   
    
    
def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('rs_filter', anonymous=True)

    rospy.Subscriber("/inflated_pseudo_scan", LaserScan, callback)    #esme topic

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
pub = rospy.Publisher('/rs_filter', LaserScan, queue_size=10)



if __name__ == '__main__':
    listener()



