#!/usr/bin/env python
import rospy
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


CHAIR_SCALE_CONST = 2.0  # Max velocity for wheelchair platform
MAX_ANGULAR_SPEED = 0.85 # Max angular velocity for wheelchair platform
OUTPUT_SCALE_FIX = 0.916


class Fuzzy1:
    """
    Produces linear velocity considering the danger factor.
    """
    def __init__(self):
        self.ang_velocity_val = 0.0
        self.real_lin_vel_val = 0.0
        self.danger_factor_val = 0.0
        self.output = 0

        # Fuzzy input(s) and output(s)
        self.risk_factor = ctrl.Antecedent(np.arange(0.0, 1.01, 0.01), 'risk_factor')
        self.ang_velocity = ctrl.Antecedent(np.arange(0.0, 1.01, 0.01), 'ang_velocity')
        self.real_lin_velocity = ctrl.Antecedent(np.arange(0.0, 1.01, 0.01), 'real_lin_velocity')
        self.lin_velocity = ctrl.Consequent(np.arange(0.0, 1.01, 0.01), 'lin_velocity')

        # Membership functions
        self.risk_factor['too_small'] = fuzz.trimf(self.risk_factor.universe, [0.0, 0.0, 0.25])
        self.risk_factor['small'] = fuzz.trimf(self.risk_factor.universe, [0.0, 0.25, 0.5])
        self.risk_factor['medium'] = fuzz.trimf(self.risk_factor.universe, [0.25, 0.5, 0.75])
        self.risk_factor['large'] = fuzz.trimf(self.risk_factor.universe, [0.5, 0.75, 1.0])
        self.risk_factor['too_large'] = fuzz.trimf(self.risk_factor.universe, [0.75, 1.0, 1.0])

        self.ang_velocity['min'] = fuzz.trimf(self.ang_velocity.universe, [0.0, 0.0, 0.5])
        self.ang_velocity['normal'] = fuzz.trimf(self.ang_velocity.universe, [0.0, 0.5, 1])
        self.ang_velocity['max'] = fuzz.trimf(self.ang_velocity.universe, [0.5, 1.0, 1.0])

        self.lin_velocity['too_slow'] = fuzz.trimf(self.lin_velocity.universe, [0.0, 0.0, 0.25])
        self.lin_velocity['slow'] = fuzz.trimf(self.lin_velocity.universe, [0.0, 0.25, 0.5])
        self.lin_velocity['normal'] = fuzz.trimf(self.lin_velocity.universe, [0.25, 0.5, 0.75])
        self.lin_velocity['fast'] = fuzz.trimf(self.lin_velocity.universe, [0.5, 0.75, 1.0])
        self.lin_velocity['too_fast'] = fuzz.trimf(self.lin_velocity.universe, [0.75, 1.0, 1.0])

        self.real_lin_velocity['too_slow'] = fuzz.trimf(self.lin_velocity.universe, [0.0, 0.0, 0.25])
        self.real_lin_velocity['slow'] = fuzz.trimf(self.lin_velocity.universe, [0.0, 0.25, 0.5])
        self.real_lin_velocity['normal'] = fuzz.trimf(self.lin_velocity.universe, [0.25, 0.5, 0.75])
        self.real_lin_velocity['fast'] = fuzz.trimf(self.lin_velocity.universe, [0.5, 0.75, 1.0])
        self.real_lin_velocity['too_fast'] = fuzz.trimf(self.lin_velocity.universe, [0.75,1.0, 1.0])

        # Fuzzy rules
        self.rule1_1  = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['too_slow'] 	& self.risk_factor['too_small']  , self.lin_velocity['normal'])
        self.rule1_2  = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['too_slow'] 	& self.risk_factor['small']      , self.lin_velocity['normal'])
        self.rule1_3  = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['too_slow'] 	& self.risk_factor['medium']     , self.lin_velocity['normal'])
        self.rule1_4  = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['too_slow']    & self.risk_factor['large']      , self.lin_velocity['slow'])
        self.rule1_5  = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['too_slow']    & self.risk_factor['too_large']  , self.lin_velocity['slow'])
        self.rule1_6  = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['slow']     	& self.risk_factor['too_small']  , self.lin_velocity['fast'])
        self.rule1_7  = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['slow']   		& self.risk_factor['small']      , self.lin_velocity['fast'])
        self.rule1_8  = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['slow']   		& self.risk_factor['medium']     , self.lin_velocity['fast'])
        self.rule1_9  = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['slow']   		& self.risk_factor['large']      , self.lin_velocity['normal'])
        self.rule1_10 = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['slow']     	& self.risk_factor['too_large']  , self.lin_velocity['slow'])
        self.rule1_11 = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['normal']      & self.risk_factor['too_small']  , self.lin_velocity['too_fast'])
        self.rule1_12 = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['normal']      & self.risk_factor['small']      , self.lin_velocity['too_fast'])
        self.rule1_13 = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['normal'] 		& self.risk_factor['medium']     , self.lin_velocity['too_fast'])
        self.rule1_14 = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['normal'] 		& self.risk_factor['large']      , self.lin_velocity['fast'])
        self.rule1_15 = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['normal'] 		& self.risk_factor['too_large']  , self.lin_velocity['normal'])
        self.rule1_16 = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['fast']     	& self.risk_factor['too_small']  , self.lin_velocity['too_fast'])
        self.rule1_17 = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['fast']   		& self.risk_factor['small']      , self.lin_velocity['too_fast'])
        self.rule1_18 = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['fast']   		& self.risk_factor['medium']     , self.lin_velocity['fast'])
        self.rule1_19 = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['fast']   		& self.risk_factor['large']      , self.lin_velocity['normal'])
        self.rule1_20 = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['fast']     	& self.risk_factor['too_large']  , self.lin_velocity['normal'])
        self.rule1_21 = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['too_fast']    & self.risk_factor['too_small']  , self.lin_velocity['too_fast'])
        self.rule1_22 = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['too_fast']    & self.risk_factor['small']      , self.lin_velocity['fast'])
        self.rule1_23 = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['too_fast'] 	& self.risk_factor['medium']     , self.lin_velocity['fast'])
        self.rule1_24 = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['too_fast'] 	& self.risk_factor['large']      , self.lin_velocity['normal'])
        self.rule1_25 = ctrl.Rule(self.ang_velocity['min']  & self.real_lin_velocity['too_fast'] 	& self.risk_factor['too_large']  , self.lin_velocity['normal'])

        self.rule2_1  = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['too_slow']     & self.risk_factor['too_small']  , self.lin_velocity['normal'])
        self.rule2_2  = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['too_slow']     & self.risk_factor['small']     , self.lin_velocity['normal'])
        self.rule2_3  = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['too_slow']     & self.risk_factor['medium']    , self.lin_velocity['slow'])
        self.rule2_4  = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['too_slow']     & self.risk_factor['large']     , self.lin_velocity['slow'])
        self.rule2_5  = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['too_slow']     & self.risk_factor['too_large'] , self.lin_velocity['too_slow'])
        self.rule2_6  = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['slow']         & self.risk_factor['too_small']  , self.lin_velocity['fast'])
        self.rule2_7  = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['slow']         & self.risk_factor['small']     , self.lin_velocity['fast'])
        self.rule2_8  = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['slow']         & self.risk_factor['medium']    , self.lin_velocity['normal'])
        self.rule2_9  = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['slow']         & self.risk_factor['large']     , self.lin_velocity['normal'])
        self.rule2_10 = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['slow']         & self.risk_factor['too_large'] , self.lin_velocity['slow'])
        self.rule2_11 = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['normal']       & self.risk_factor['too_small']  , self.lin_velocity['fast'])
        self.rule2_12 = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['normal']       & self.risk_factor['small']     , self.lin_velocity['fast'])
        self.rule2_13 = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['normal']       & self.risk_factor['medium']    , self.lin_velocity['normal'])
        self.rule2_14 = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['normal']       & self.risk_factor['large']     , self.lin_velocity['normal'])
        self.rule2_15 = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['normal']       & self.risk_factor['too_large'] , self.lin_velocity['slow'])
        self.rule2_16 = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['fast']         & self.risk_factor['too_small']  , self.lin_velocity['fast'])
        self.rule2_17 = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['fast']         & self.risk_factor['small']     , self.lin_velocity['fast'])
        self.rule2_18 = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['fast']         & self.risk_factor['medium']    , self.lin_velocity['normal'])
        self.rule2_19 = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['fast']         & self.risk_factor['large']     , self.lin_velocity['slow'])
        self.rule2_20 = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['fast']         & self.risk_factor['too_large'] , self.lin_velocity['slow'])
        self.rule2_21 = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['too_fast']     & self.risk_factor['too_small']  , self.lin_velocity['too_fast'])
        self.rule2_22 = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['too_fast']     & self.risk_factor['small']     , self.lin_velocity['fast'])
        self.rule2_23 = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['too_fast']     & self.risk_factor['medium']    , self.lin_velocity['normal'])
        self.rule2_24 = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['too_fast']     & self.risk_factor['large']     , self.lin_velocity['normal'])
        self.rule2_25 = ctrl.Rule(self.ang_velocity['normal']  & self.real_lin_velocity['too_fast']     & self.risk_factor['too_large'] , self.lin_velocity['normal'])

        self.rule3_1  = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['too_slow'] & self.risk_factor['too_small']  , self.lin_velocity['slow'])
        self.rule3_2  = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['too_slow'] & self.risk_factor['small']      , self.lin_velocity['slow'])
        self.rule3_3  = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['too_slow'] & self.risk_factor['medium']     , self.lin_velocity['slow'])
        self.rule3_4  = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['too_slow'] & self.risk_factor['large']      , self.lin_velocity['too_slow'])
        self.rule3_5  = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['too_slow'] & self.risk_factor['too_large']  , self.lin_velocity['too_slow'])
        self.rule3_6  = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['slow']     & self.risk_factor['too_small']  , self.lin_velocity['normal'])
        self.rule3_7  = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['slow']     & self.risk_factor['small']      , self.lin_velocity['normal'])
        self.rule3_8  = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['slow']     & self.risk_factor['medium']     , self.lin_velocity['slow'])
        self.rule3_9  = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['slow']     & self.risk_factor['large']      , self.lin_velocity['too_slow'])
        self.rule3_10 = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['slow']     & self.risk_factor['too_large']  , self.lin_velocity['too_slow'])
        self.rule3_11 = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['normal']   & self.risk_factor['too_small']  , self.lin_velocity['normal'])
        self.rule3_12 = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['normal']   & self.risk_factor['small']      , self.lin_velocity['normal'])
        self.rule3_13 = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['normal']   & self.risk_factor['medium']     , self.lin_velocity['slow'])
        self.rule3_14 = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['normal']   & self.risk_factor['large']      , self.lin_velocity['too_slow'])
        self.rule3_15 = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['normal']   & self.risk_factor['too_large']  , self.lin_velocity['too_slow'])
        self.rule3_16 = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['fast']     & self.risk_factor['too_small']  , self.lin_velocity['fast'])
        self.rule3_17 = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['fast']     & self.risk_factor['small']      , self.lin_velocity['normal'])
        self.rule3_18 = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['fast']     & self.risk_factor['medium']     , self.lin_velocity['normal'])
        self.rule3_19 = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['fast']     & self.risk_factor['large']      , self.lin_velocity['slow'])
        self.rule3_20 = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['fast']     & self.risk_factor['too_large']  , self.lin_velocity['slow'])
        self.rule3_21 = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['too_fast'] & self.risk_factor['too_small']  , self.lin_velocity['fast'])
        self.rule3_22 = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['too_fast'] & self.risk_factor['small']      , self.lin_velocity['fast'])
        self.rule3_23 = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['too_fast'] & self.risk_factor['medium']     , self.lin_velocity['normal'])
        self.rule3_24 = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['too_fast'] & self.risk_factor['large']      , self.lin_velocity['normal'])
        self.rule3_25 = ctrl.Rule(self.ang_velocity['max']  & self.real_lin_velocity['too_fast'] & self.risk_factor['too_large']  , self.lin_velocity['normal'])

        # Fuzzy control system and control system simulator
        self.lin_velocity_ctrl = ctrl.ControlSystem([self.rule1_1, self.rule1_2, self.rule1_3, self.rule1_4, self.rule1_5, self.rule1_6, self.rule1_7, self.rule1_8, self.rule1_9, self.rule1_10, self.rule1_11, self.rule1_12, self.rule1_13, self.rule1_14, self.rule1_15, self.rule1_16, self.rule1_17, self.rule1_18, self.rule1_19, self.rule1_20, self.rule1_21, self.rule1_22, self.rule1_23, self.rule1_24, self.rule1_25,
                                               		 self.rule2_1, self.rule2_2, self.rule2_3, self.rule2_4, self.rule2_5, self.rule2_6, self.rule2_7, self.rule2_8, self.rule2_9, self.rule2_10, self.rule2_11, self.rule2_12, self.rule2_13, self.rule2_14, self.rule2_15, self.rule2_16, self.rule2_17, self.rule2_18, self.rule2_19, self.rule2_20, self.rule2_21, self.rule2_22, self.rule2_23, self.rule2_24, self.rule2_25,
                                                	 self.rule3_1, self.rule3_2, self.rule3_3, self.rule3_4, self.rule3_5, self.rule3_6, self.rule3_7, self.rule3_8, self.rule3_9, self.rule3_10, self.rule3_11, self.rule3_12, self.rule3_13, self.rule3_14, self.rule3_15, self.rule3_16, self.rule3_17, self.rule3_18, self.rule3_19, self.rule3_20, self.rule3_21, self.rule3_22, self.rule3_23, self.rule3_24, self.rule3_25])

        self.lin_velocity_sim = ctrl.ControlSystemSimulation(self.lin_velocity_ctrl, clip_to_bounds=True)
        print("--> Linear Velocity Planner Fuzzy-1 Initiated!")
        # rospy.sleep(1) # Why is this in here, reconsider it durint tests


    def run(self):
        """
        Appy inputs to inference system and compute output
        """
        self.lin_velocity_sim.input['ang_velocity'] = self.ang_velocity_val
        self.lin_velocity_sim.input['real_lin_velocity'] = self.real_lin_vel_val
        self.lin_velocity_sim.input['risk_factor'] = self.danger_factor_val
        self.lin_velocity_sim.compute()

        self.output = self.lin_velocity_sim.output['lin_velocity']


    def danger_factor_callback(self, msg):
        """
        Set danger factor value from the ROS message - danger factor node
        """
        self.danger_factor_val = msg.data


    def real_lin_vel_callback(self, msg):
        """
        Set real linear velocity and angular velocity
        """
        # self.real_lin_vel_val = msg.linear.x / CHAIR_SCALE_CONST
        # self.ang_valocity_val = abs(msg.angular.z) / MAX_ANGULAR_SPEED
        self.real_lin_vel_val = msg.twist.twist.linear.x / CHAIR_SCALE_CONST
        self.ang_velocity_val = abs(msg.twist.twist.angular.z) / MAX_ANGULAR_SPEED

        if self.ang_velocity_val > MAX_ANGULAR_SPEED:
            self.danger_factor_val = 1


if __name__ == '__main__':
    
    rospy.init_node('fuzzy_velocity_planner', anonymous=True)

    fuzzy1 = Fuzzy1() # Deafault parameter is 160 kg

    lin_vel_output_pub = rospy.Publisher('/fuzzy_lin_vel', Float32, queue_size=10)
    
    rospy.Subscriber('/danger_factor_output', Float32, fuzzy1.danger_factor_callback)
    rospy.Subscriber('/odom', Odometry, fuzzy1.real_lin_vel_callback)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        fuzzy1.run()
        output = fuzzy1.output / OUTPUT_SCALE_FIX
        lin_vel_output_pub.publish(output * CHAIR_SCALE_CONST)

        # print("Velocity Output (0-1 Range)          :{:.5f}".format(fuzzy1.output))
        # print("Velocity Output (for WCP)            :{:.5f}".format(fuzzy1.output * CHAIR_SCALE_CONST))
        # print()

        rate.sleep()

