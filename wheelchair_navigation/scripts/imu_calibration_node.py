#!/usr/bin/env python
import rospy
import ros_numpy
#from .registry import converts_from_numpy, converts_to_numpy
import numpy as np
import geometry_msgs.msg
import geometry_msgs
import std_msgs.msg
#from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
from roslib import message
from sensor_msgs.msg import Imu
import pcl
#import pcl_ros
import ctypes
import struct
from std_msgs.msg import Header
from pcl_msgs.msg import ModelCoefficients


class calibration:
    def __init__(self):
        #self.orientation = None
        #self.velocity = None
        self.Imu_list_x = []
        self.Imu_list_y = []
        self.Imu_list_z = []
        self.coef_list_x = []
        self.coef_list_y = []
        self.coef_list_z = []
        self.coef_list_d = []
        self.prepared = False
        self.prepared_IMU = False
        self.prepared_IMU_pub = False
        self.prepared_coeff = False
        self.prepared_coeff_pub = False
        self.mean_x = None
        self.mean_y = None
        self.mean_z = None
        self.mean_coef_x = None
        self.mean_coef_y = None
        self.mean_coef_z = None
        self.mean_coef_d = None

    
    def plane_coeff(self,msg):
        global coefx,coefy,coefz,coefd 
        # coef=msg
        coefx = msg.values[0]  #0x
        coefy = msg.values[1]  #1y
        coefz = msg.values[2]  #2z
        coefd = msg.values[3]  #3d


        if not self.prepared_coeff:
            self.coef_list_x.append(coefx)
            self.coef_list_y.append(coefy)
            self.coef_list_z.append(coefz)
            self.coef_list_d.append(coefd)
        
            if (len(self.coef_list_x) > 100):
                self.prepared_coeff = True 
            
        else:

            if  self.prepared_coeff_pub == False :
                self.mean_coef_x=np.mean(np.asarray(self.coef_list_x))
                self.mean_coef_y=np.mean(np.asarray(self.coef_list_y))
                self.mean_coef_z=np.mean(np.asarray(self.coef_list_z))
                self.mean_coef_d=np.mean(np.asarray(self.coef_list_d))
                self.prepared_coeff_pub = True

            else : 
                msg.values = (self.mean_coef_x, self.mean_coef_y, self.mean_coef_z, self.mean_coef_d)
            


            # plt.figure()
            # plt.grid()
            # plt.plot(self.coef_list_y,linewidth=2.0,label='coef_list_Y')
            # plt.plot([0,100],[mean_coef_y,mean_coef_y],linewidth=3.0, color='red' ,label='Mean')
            # plt.ylabel('coef_list_Y ')
            # plt.yticks(np.arange(min(self.coef_list_y), max(self.coef_list_y), 0.005))
            # plt.legend()
            # plt.xlabel('number of data ')

            # plt.figure()
            # plt.grid()
            # plt.plot(range(0,len(self.coef_list_z)),self.coef_list_z,linewidth=2.0,label='coef_list_Z')
            # plt.plot([0,100],[mean_coef_z,mean_coef_z],linewidth=3.0, color='red',label='Mean')
            # plt.ylabel('coef_list_Z ')
            # plt.xlabel('number of data ')
            # plt.yticks(np.arange(min(self.coef_list_z), max(self.coef_list_z), 0.005))
            # plt.legend()

            # plt.figure()
            # plt.grid()
            # plt.plot(range(0,len(self.coef_list_x)),self.coef_list_x,linewidth=2.0,label='coef_list_X')
            # plt.plot([0,100],[mean_coef_x,mean_coef_x],linewidth=3.0, color='red',label='Mean')
            # plt.legend()
            # plt.ylabel('coef_list_x ')
            # plt.xlabel('number of data ')
            # plt.yticks(np.arange(min(self.coef_list_x), max(self.coef_list_x), 0.005))


            # plt.figure()
            # plt.grid()
            # plt.plot(range(0,len(self.coef_list_d)),self.coef_list_d,linewidth=2.0,label='coef_list_d')
            # plt.plot([0,100],[mean_coef_d,mean_coef_d],linewidth=3.0, color='red',label='Mean')
            # plt.legend()
            # plt.ylabel('coef_list_d ')
            # plt.xlabel('number of data ')
            # plt.yticks(np.arange(min(self.coef_list_d), max(self.coef_list_d), 0.05))



            # plt.show()  
        pub2.publish(msg)




    def preparing(self,msg):

        self.imu_msg = msg
        
        if not self.prepared_IMU:
            self.Imu_list_x.append(msg.linear_acceleration.x)
            self.Imu_list_y.append(msg.linear_acceleration.y)
            self.Imu_list_z.append(msg.linear_acceleration.z)
        
            if (len(self.Imu_list_x) > 300):
                self.prepared_IMU = True 
            
        else:
            if  self.prepared_IMU_pub == False :
                self.mean_x=np.mean(np.asarray(self.Imu_list_x))
                self.mean_y=np.mean(np.asarray(self.Imu_list_y))
                self.mean_z=np.mean(np.asarray(self.Imu_list_z))
                self.prepared_IMU_pub = True
            else:
                msg.linear_acceleration.x = self.mean_x
                msg.linear_acceleration.y = self.mean_y
                msg.linear_acceleration.z = self.mean_z
            # plt.figure()
            # plt.grid()
            # plt.plot(self.Imu_list_y,linewidth=2.0,label='linear acceleration Y')
            # plt.plot([0,300],[mean_y,mean_y],linewidth=3.0, color='red' ,label='Mean')
            # plt.ylabel('linear acceleration Y ')
            # plt.yticks(np.arange(min(self.Imu_list_y), max(self.Imu_list_y), 0.005))
            # plt.legend()
            # plt.xlabel('number of data ')

            # plt.figure()
            # plt.grid()
            # plt.plot(range(0,len(self.Imu_list_z)),self.Imu_list_z,linewidth=2.0,label='linear acceleration Z')
            # plt.plot([0,300],[mean_z,mean_z],linewidth=3.0, color='red',label='Mean')
            # plt.ylabel('linear acceleration Z ')
            # plt.xlabel('number of data ')
            # plt.yticks(np.arange(min(self.Imu_list_z), max(self.Imu_list_z), 0.005))
            # plt.legend()

            # plt.figure()
            # plt.grid()
            # plt.plot(range(0,len(self.Imu_list_x)),self.Imu_list_x,linewidth=2.0,label='linear acceleration X')
            # plt.plot([0,300],[mean_x,mean_x],linewidth=3.0, color='red',label='Mean')
            # plt.legend()
            # plt.ylabel('linear acceleration X ')
            # plt.xlabel('number of data ')
            # plt.yticks(np.arange(min(self.Imu_list_x), max(self.Imu_list_x), 0.005))

            # plt.show()    
        
        
        pub.publish(msg)



if __name__ == '__main__':
    
    rospy.init_node('imu_calibration', anonymous=True)

    cl = calibration()
    rospy.Subscriber("/camera/accel/sample", Imu, cl.preparing)    #esme topic
    rospy.Subscriber("/plane_coeff", ModelCoefficients, cl.plane_coeff)
    pub = rospy.Publisher('Imu_calibration', Imu, queue_size=10)
    pub2 = rospy.Publisher('model_coeff_calibrated', ModelCoefficients, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    rospy.spin()
    rate.sleep()
   