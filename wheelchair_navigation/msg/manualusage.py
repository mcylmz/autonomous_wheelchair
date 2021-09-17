import rospy
from chair.msg import MotorReference
import copy

v=0.3
w=0
   
if __name__ == '__main__':
   publisher1= rospy.Publisher('/motor_ref', MotorReference, queue_size = 10)
   rospy.init_node('manual_node')
   rate=rospy.Rate(10)
   while not rospy.is_shutdown():
      ratio=32
      L=0.2975
      r=0.2
      v1=v+L*w
      v2=v-L*w
      v1rpm=(v1*60)/(2*mth.pi*r)
      v2rpm=(v2*60)/(2*mth.pi*r)
      v1motrpm=v1rpm*ratio
      v2motrpm=v2rpm*ratio
      v1motref=v1motrpm/2
      v2motref=v2motrpm/2
      motors=MotorReference()
      motors.ref1=v1motref
      motors.ref2=v2motref
      publisher1.publish(motors)
      rate.sleep()
