#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


SAMPLE_SIZE = 108
SAMPLE_SIZE_2 = 54
UPPER_BOUND = 54


class Risk:
    """
    Class for storing risk variables
    """
    def __init__(self):
        self.rads = np.empty(SAMPLE_SIZE_2)
        self.ranges_left = np.empty(SAMPLE_SIZE_2)
        self.ranges_right = np.empty(SAMPLE_SIZE_2)
        self.risk_left = np.empty(SAMPLE_SIZE_2)
        self.risk_right = np.empty(SAMPLE_SIZE_2)
        self.total_risk = 0
        self.risk_max = np.empty(SAMPLE_SIZE_2)
        self.left_max = np.empty(SAMPLE_SIZE_2)
        self.right_max = np.empty(SAMPLE_SIZE_2)
        self.risk = np.empty(SAMPLE_SIZE)
        self.risk_idx_left = []
        self.risk_idx_rigth = []
        self.s = 5 # 5 # Forward
        self.k = 20 # 8 # Lateral
        self.y = 2 # 3
        self.max = 0
        self.bell_func = np.empty(SAMPLE_SIZE_2)


def callback(msg):
    """
    LaserScan callback to set risk parameters
    """
    global risk

    risk.ranges_right = msg.ranges[:UPPER_BOUND]
    risk.ranges_right = np.array(risk.ranges_right[::-1])
    risk.ranges_left = np.array(msg.ranges[UPPER_BOUND:])

    for i in range(len(msg.ranges) / 2):
        risk.rads[i] = msg.angle_increment * i

        # Range saturations
        # If greater than max range
        if risk.ranges_right[i] >= msg.range_max:
            risk.ranges_right[i] = msg.range_max
        
        if risk.ranges_left[i] >= msg.range_max:
            risk.ranges_left[i] = msg.range_max

        # If lower than min range
        if risk.ranges_right[i] < msg.range_min:
            risk.ranges_right[i] = msg.range_min
        
        if risk.ranges_left[i] < msg.range_min:
            risk.ranges_left[i] = msg.range_min

    for i in range(len(msg.ranges) / 2):
        risk.bell_func[i] = 1 / (((risk.rads[i] / risk.k) ** risk.y) + 1)
        
        risk.risk_left[i] = 1 - ((risk.ranges_left[i] / msg.range_max) ** risk.s)
        risk.left_max[i] = risk.risk_left[i] * risk.bell_func[i]

        risk.risk_right[i] = 1 - ((risk.ranges_right[i] / msg.range_max) ** risk.s)
        risk.right_max[i] = risk.risk_right[i] * risk.bell_func[i]

    risk.max = max(np.amax(risk.right_max), np.amax(risk.left_max))
    risk.max = np.round(risk.max, 3)
    # print('Maximum danger value: ' + str(risk.max))


if __name__ == '__main__':

    global risk
    risk = Risk()

    rospy.init_node('danger_factor_node', anonymous=True)

    rospy.Subscriber('/inflated_pseudo_scan', LaserScan, callback) 
    
    danger_factor_pub = rospy.Publisher('/danger_factor_output', Float32, queue_size=10)

    rate = rospy.Rate(50)
    
    while not rospy.is_shutdown():
        danger_factor_pub.publish(risk.max)
        rate.sleep()



