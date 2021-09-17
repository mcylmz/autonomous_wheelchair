#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
from geometry_msgs.msg import Point,Twist
# global a
class H:
    def __init__(self):
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.input)
        self.pub = rospy.Publisher('/go_p', Twist, queue_size=20)  
        self.timer = rospy.Timer(rospy.Duration(0.0125), self.output)
        self.go_pub = None
        self.in_sig = Twist()
    
    def input(self,msg):
        pass
        # print "input"
        self.in_sig=msg
        # self.output()
    def output(self,timer):
        pass
        # print "output"
        self.pub.publish(self.in_sig)

if __name__ == '__main__':
    # global a
    rospy.init_node('go_node', anonymous=True)
    rate = rospy.Rate(60)
    H()
    # rospy.Subscriber("/cmd_vel", Twist, class_cb.input)
    # class_cb.output()
    # rospy.Timer(rospy.Duration(20), class_cb.input)
    # rospy.Timer(rospy.Duration(20), class_cb.output)

    rospy.spin()