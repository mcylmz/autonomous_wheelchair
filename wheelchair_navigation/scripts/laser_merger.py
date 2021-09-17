#!/usr/bin/env python
import rospy
import copy
import numpy as np
import tf

from sensor_msgs.msg import LaserScan



class LaserScanMerger:
    
    def __init__(self):
        self.rp_lidar_msg = None
        self.rs_lidar_msg = None
        self.merged_lidar = []
        self.transfom_listener = tf.TransformListener()

        # All angles in radians
        self.rp_angle_min = None
        self.rp_angle_max = None
        self.rp_angle_inc = None
        self.rs_angle_min = None
        self.rs_angle_max = None
        self.rs_angle_inc = None
    

    def rp_lidar_callback(self, msg):
        """
        Callback function to store RPLiDAR ROS message.
        """
        self.rp_lidar_msg = []

        for j in msg.ranges:
            self.rp_lidar_msg.append(self.transfom_listener.transformPoint("base_link", ))


        self.rp_lidar_msg.range_max = 2.0 # range_max in incoming message was 18.0
        self.rp_angle_min = self.rp_lidar_msg.angle_min
        self.rp_angle_max = self.rp_lidar_msg.angle_max
        self.rp_angle_inc = self.rp_lidar_msg.angle_increment


    def rs_lidar_callback(self, msg):
        """
        Callback function to store RealSense LiDAR ROS message.
        """
        self.rs_lidar_msg = msg
        self.rs_angle_min = self.rs_lidar_msg.angle_min
        self.rs_angle_max = self.rs_lidar_msg.angle_max
        self.rs_angle_inc = self.rs_lidar_msg.angle_increment

        self.laser_scan_merger()


    def laser_scan_merger(self):
        try:
            self.merged_lidar = copy.deepcopy(list(self.rp_lidar_msg.ranges))
        except:
            print("Nonetype exception occured.")

        rs_lidar_ranges = list(self.rs_lidar_msg.ranges)
        rs_lidar_angle_list = [round((self.rs_angle_min + i * self.rs_angle_inc), 3) for i in range(len(rs_lidar_ranges))]
        rs_lidar_angle_array = np.array(rs_lidar_angle_list)

        for i in range(len(self.merged_lidar)):
            rp_angle_temp = round(self.rp_angle_min + i * self.rp_angle_inc,3)

            # if not ((rp_angle_temp < self.rs_angle_min) and ((rp_angle_temp > self.rs_angle_max))):
            if  i <= np.floor(len(self.merged_lidar)/2):
                if i == 54:
                    ali =1
                if rp_angle_temp > self.rs_angle_min:
                    absolute_val_array = np.abs(rs_lidar_angle_array - rp_angle_temp)
                    smallest_difference_index = absolute_val_array.argmin()
                    print("Index: %d" % i)
                    print("Diff index: %d" % smallest_difference_index)
                    
                    if (np.isnan(self.merged_lidar[i]) or np.isnan(rs_lidar_ranges[smallest_difference_index])):
                        if (np.isnan(self.merged_lidar[i]) and (not (np.isnan(rs_lidar_ranges[smallest_difference_index])))):
                            print("rp: nan, rs: not nan (%d)" % i)
                            self.merged_lidar[i] = rs_lidar_ranges[smallest_difference_index]
                        elif (np.isnan(rs_lidar_ranges[smallest_difference_index])) and ((np.isnan(self.merged_lidar[i]))):
                            print("rp: nan, rs: nan (%d)" % i)
                            self.merged_lidar[i] = float('inf')                 
                        else: # if RealSense data is nan
                            print("rp: not nan, rs: nan")
                    else:
                        if self.merged_lidar[i] >= rs_lidar_ranges[smallest_difference_index]:
                            self.merged_lidar[i] = rs_lidar_ranges[smallest_difference_index]
                            # print("HATA")
            else:       #i == np.floor(len(self.merged_lidar)/2):
                
                 if rp_angle_temp < self.rs_angle_max:
                    absolute_val_array = np.abs(rs_lidar_angle_array - rp_angle_temp)
                    smallest_difference_index = absolute_val_array.argmin()
                    print("Index: %d" % i)
                    print("Diff index: %d" % smallest_difference_index)
                    
                    if (np.isnan(self.merged_lidar[i]) or np.isnan(rs_lidar_ranges[smallest_difference_index])):
                        if (np.isnan(self.merged_lidar[i]) and (not (np.isnan(rs_lidar_ranges[smallest_difference_index])))):
                            print("rp: nan, rs: not nan (%d)" % i)
                            self.merged_lidar[i] = rs_lidar_ranges[smallest_difference_index]
                        elif (np.isnan(rs_lidar_ranges[smallest_difference_index])) and ((np.isnan(self.merged_lidar[i]))):
                            print("rp: nan, rs: nan (%d)" % i)
                            self.merged_lidar[i] = float('inf')                 
                        else: # if RealSense data is nan
                            print("rp: not nan, rs: nan")
                    else:
                        if self.merged_lidar[i] >= rs_lidar_ranges[smallest_difference_index]:
                            self.merged_lidar[i] = rs_lidar_ranges[smallest_difference_index]
                            # print("HATA")

        try:
            merged_output_laser = copy.deepcopy(self.rp_lidar_msg)
            time_now = rospy.Time.now()
            merged_output_laser.header.stamp.nsecs = time_now.nsecs
            merged_output_laser.header.stamp.secs  = time_now.secs
            merged_output_laser.ranges = tuple(self.merged_lidar)
            merged_output_laser.intensities = tuple()
            pub.publish(merged_output_laser)
        except:
            print("Nonetype exception occured.")


if __name__ == '__main__':
   
    rospy.init_node('laser_merge', anonymous=True)
  
    laser_merger = LaserScanMerger()
   
    pub = rospy.Publisher('/laser_merge', LaserScan, queue_size=1)
    
    rospy.Subscriber("/front_rp/rp_scan_filtered_front", LaserScan, laser_merger.rp_lidar_callback)    
    rospy.Subscriber("/rs_filtered", LaserScan, laser_merger.rs_lidar_callback)    
    # rospy.Subscriber("/tb3_scan", LaserScan, server.lidar_callback)                                                           
    # pub2 = rospy.Publisher('/focm_output', Float32MultiArray, queue_size=1)
    rate = rospy.Rate(100) # 10hz
    rospy.spin()