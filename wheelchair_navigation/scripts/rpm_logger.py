#!/usr/bin/env python
import rospy
from wheelchair_navigation.msg import MotorMonitor


class Logger:
    def __init__(self):
        self.rpm_left = 0.
        self.rmp_right = 0.

    
    def rpm_cb(self, msg):
        log = str(msg.rpm1) + " " + str(msg.rpm2) + "\n"
        file = open("rpm_1ms.txt", "a")
        file.write(log)
        file.close()


if __name__ == "__main__":
    rospy.init_node("rpm_logger", anonymous=True)
    logger = Logger()

    rospy.Subscriber("/motor_monitor", MotorMonitor, logger.rpm_cb)

    rospy.spin()