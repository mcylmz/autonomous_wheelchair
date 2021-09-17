#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np
import move_base_msgs.msg 
from nav_msgs.msg import Path
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, PoseStamped, Twist, Vector3, Pose, Quaternion, PoseWithCovarianceStamped


class Server:
    
    def __init__(self):
        
        
        self.time_now = None
        self.linear_vel_ref = None
        self.angular_vel_ref = None
        
        self.odom_yaw = None
        self.odom_yaw_amcl = None

        self.odom_r_x = None
        self.odom_r_y = None
   

    def clock_cb(self,msg):

        mili_sec = round((float(msg.clock.nsecs)/1000000000), 1)
        self.time_now = msg.clock.secs + mili_sec
       
        
    def odom_callback(self,msg):

        self.odom_r_x = round(msg.pose.pose.position.x,4)
        self.odom_r_y = round(msg.pose.pose.position.y,4)
        odom_orientation_list = [msg.pose.pose.orientation.x ,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,msg.pose.pose.orientation.w ]
        (odom_roll,odom_pich,self.odom_yaw) = euler_from_quaternion(odom_orientation_list)
         
    def amcl_callback(self,msg):

        self.odom_r_x_amcl = round(msg.pose.pose.position.x,4)
        self.odom_r_y_amcl = round(msg.pose.pose.position.y,4)
        odom_orientation_list_amcl = [msg.pose.pose.orientation.x ,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,msg.pose.pose.orientation.w ]
        (odom_roll,odom_pich,self.odom_yaw_amcl) = euler_from_quaternion(odom_orientation_list_amcl)
    
    def cmd_callback(self,msg):
        self.linear_vel_ref = msg.linear.x
        self.angular_vel_ref = msg.angular.z
        

    def lidar_callback(self,msg):

        x=list(msg.ranges)

        # save_temp is our memory
        save_temp = list()
        save_temp.append(self.time_now)

        #if you use both odom and AMCL comment: the following lines:
        # save_temp.append(1995)
        # save_temp.append(1995)
        # save_temp.append(1995)
        
        #if you use one of the AMCL or ODOM you should hold previous lines and comment the following lines which you do not use:
        #for AMCL
        save_temp.append(self.odom_r_x_amcl)
        save_temp.append(self.odom_r_y_amcl)
        save_temp.append(self.odom_yaw_amcl)
        #for odom
        save_temp.append(self.odom_r_x)
        save_temp.append(self.odom_r_y)
        save_temp.append(self.odom_yaw)

        #adding speed ref:
        save_temp.append(self.linear_vel_ref)
        save_temp.append(self.angular_vel_ref)
      
        for i in range(0,(len(msg.ranges))):
            if math.isinf(x[i]):    # using 999 instead of inf will prevent errors in Matlab :D at least I think :P
                lidar_temp = 999
            else:
                lidar_temp = x[i]
            save_temp.insert(9+i,lidar_temp)
            #upto now the save temp should be something like this:

            #save_temp = [time, x_amcl, y_amcl, yaw_amcl, odom_r_x, odom_r_y, odom_yaw, linear_speed, angular_speed, laser 0, laser 1, ..., laser end]
            
            #now we are going to save this to a text file.

        #save mode change the address below:
        a = open("/home/dell/catkin_ws/src/topic_logger/scripts/fuzzy_speed.txt", 'a')
        a.write(str(save_temp))
        a.write(str("\n"))
        a.close
        ccc=1


if __name__ == '__main__':
    
    rospy.init_node('obstacle_record', anonymous=True)
    server = Server()
    rospy.Subscriber("/clock", Clock, server.clock_cb)
    
    #if you want AMCL -> you should comment ODOM subscriber
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, server.amcl_callback) #AMCL subscriber
    #if you want ODOM -> you should comment AMCL subscriber
    rospy.Subscriber("/odom", Odometry, server.odom_callback) #ODOM subscriber

    rospy.Subscriber("/cmd_vel", Twist, server.cmd_callback) #cmd subscriber
    rospy.Subscriber("/inflated_pseudo_scan", LaserScan, server.lidar_callback)    

    rate = rospy.Rate(50) # 10hz
    rospy.spin()

