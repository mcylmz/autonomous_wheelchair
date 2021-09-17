#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import UInt8
from std_msgs.msg import Float32
from chair.msg import MotorMonitor

def callback(msg):
    # rospy.loginfo("encoder1 %s",msg.encoder1)
    # rospy.loginfo(msg.encoder2)
    # rospy.loginfo(msg.amps1)
    # rospy.loginfo(msg.amps2)
    # rospy.loginfo(msg.rpm1)
    # rospy.loginfo(msg.rpm2)
    # rospy.loginfo(msg.cmd1)
    # rospy.loginfo(msg.cmd2)
    # rospy.loginfo(msg.faultFlags)

    enc1=Int32()
    enc2=Int32()
    I1=Float32()
    I2=Float32()
    sp1=Int32()
    sp2=Int32()
    rf1=Int32()
    rf2=Int32()
    fflags=UInt8()


    enc1.data=msg.encoder1
    enc2.data=msg.encoder2
    I1.data=msg.amps1
    I2.data=msg.amps2
    sp1.data=msg.rpm1
    sp2.data=msg.rpm2
    rf1.data=msg.cmd1
    rf2.data=msg.cmd2
    fflags.data=msg.faultFlags

    pub.publish(enc1)
    pub2.publish(enc2)
    pub3.publish(I1)
    pub4.publish(I2)
    pub5.publish(sp1)
    pub6.publish(sp2)
    pub7.publish(rf1)
    pub8.publish(rf2)
    pub9.publish(fflags)

    
def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('hosein_sub', anonymous=True)

    rospy.Subscriber("motor_monitor", MotorMonitor, callback)    #esme topic

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
pub = rospy.Publisher('enc1', Int32, queue_size=10)
pub2 = rospy.Publisher('enc2', Int32, queue_size=10)
pub3 = rospy.Publisher('I1', Float32, queue_size=10)
pub4 = rospy.Publisher('I2', Float32, queue_size=10)
pub5 = rospy.Publisher('sp1', Int32, queue_size=10)
pub6 = rospy.Publisher('sp2', Int32, queue_size=10)
pub7 = rospy.Publisher('rf1', Int32, queue_size=10)
pub8 = rospy.Publisher('rf2', Int32, queue_size=10)
pub9 = rospy.Publisher('fflags', UInt8, queue_size=10)


if __name__ == '__main__':
    listener()



