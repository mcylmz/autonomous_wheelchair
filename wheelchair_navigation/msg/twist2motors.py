#!/usr/bin/env python
"""
   twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
   
   
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import rospy
import roslib
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 
from wheelchair_navigation.msg import SpeedReference


class TwistToMotors():

    def __init__(self):
        rospy.init_node("twist_to_motors")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
    
        self.width = rospy.get_param("~base_width", 0.59)
    
        #self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32,queue_size=10)
        #self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32,queue_size=10)
        
        self.publisher1 = rospy.Publisher('/speed_ref', SpeedReference, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.twistCallback)
    
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)

        self.left = 0
        self.right = 0

        self.a = 0
        self.b = 0
        

    def spin(self):
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(50)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
    
        ###### main loop  ######
        while not rospy.is_shutdown():
        
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()
                

    def spinOnce(self):
        # dx = (l + r) / 2
        # dr = (r - l) / width
            
        self.right = 1.0 * self.dx + self.dr * self.width / 2 # [m/s] -> v_r = v + w * L
        self.left = 1.0 * self.dx - self.dr * self.width / 2  # [m/s] -> v_r = v - w * L
        # rospy.loginfo("publishing: (%d, %d)", left, right) 
                
        # self.pub_lmotor=self.left
        # self.pub_rmotor=self.right
        N = 32
        wheel_diameter = 0.380
        pi = 3.141592654 
       
        self.a = (self.left * N * 60) / (wheel_diameter * (2 * pi))  # [rpm]
        self.b = (self.right * N * 60) / (wheel_diameter * (2 * pi))  # [rpm]

        # Chairnode expects values in range [0, 1000]. So the this node must not
        # publish values in RPM. It must publish RPM percentage considering motor
        # driver inputs in range [0, 1000]. Below code snippet converts RPM values
        # into RPM percentage. (MAX RPM = 1500 => Command Value = 1000)
        self.a = self.a * (1000 / 1500.)
        self.b = self.b * (1000 / 1500.)
        # rospy.loginfo("a: %f" % self.a)
        # rospy.loginfo("b: %f" % self.b)
        
        if self.a > 1000:
            self.a = 1000
        
        if self.b > 1000:
            self.b = 1000
        
        # No backward motion for wheelchair
        # Below saturations are unuseful for now | 22.04.2021
        if self.a < -1000:
            self.a = -1000

        if self.b < -1000:
            self.b = -1000
        
        if self.a < 0:
            self.a = 0
        elif 100 > self.a >= 15 :
            self.a = 100 
        
        if self.b < 0:
            self.b = 0
        elif 100 > self.b >= 15 :
            self.b = 100 

        speeds = SpeedReference()
        speeds.left = int(self.a)
        speeds.right = int(self.b)

        try:
            self.publisher1.publish(speeds)
        except ValueError:
            pass


    def twistCallback(self,msg):
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
    

if __name__ == '__main__':

    twistToMotors = TwistToMotors()
    twistToMotors.spin()
