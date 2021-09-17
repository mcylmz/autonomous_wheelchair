#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

def callback(msg):
    L_data = []
    L_data = list(msg.ranges)
    L_data_i = []
    L_data_i = list(msg.intensities)
    hi=[]
    hi2=[]
    for i in range (0,(182-102)):
        hi.append(np.inf)
    for i in range (0,(182-102)):
        hi2.append(0)
    L_data[104:179] = hi
    L_data_i[104:179] = hi2
    msg.ranges = tuple(L_data)
    msg.intensities = tuple(L_data_i)
    # msg.header.frame_id = "go"
    a=1
    pub.publish(msg)
    # rate = rospy.Rate(20)
    rate.sleep()
   
def listener():
    rospy.Subscriber("/scan_rp", LaserScan, callback)    #esme topic
pub = rospy.Publisher('/hosein_scan', LaserScan, queue_size=10)

if __name__ == '__main__':
    # rate = rospy.Rate(20)
    rospy.init_node('rp_scan_fixed', anonymous=True)
    rate = rospy.Rate(100)
    listener()
    rospy.spin()
     


