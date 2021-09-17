#!/usr/bin/env python
import rospy
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class Fuzzy2:
    """
    Produces mass scaling factor
    """
    def __init__(self, mass_total):
        # Fuzzy input(s) and output(s)
        self.mass = ctrl.Antecedent(np.arange(70, 201, 1), 'mass')
        self.ref_angular_vel = ctrl.Antecedent(np.arange(0.0, 1.01, 0.01), 'ref_angular_vel')
        self.mass_scaling_factor = ctrl.Consequent(np.arange(0.0, 1.01, 0.01), 'mass_scaling_factor')

        # Variable definitions
        self.mass_input = mass_total # person + wheelchair
        self.ref_angular_vel_val = 0
        self.mass_scaling_factor_val = 0
        self.output = 0
        self.ang_vel_max = 0.85

        self.last_callback_time = 0
        self.callback_time = 0
        self.sample_time = 0

        self.goal_deadzone = 0.5
        self.distance_to_goal = 0

        # Membership functions
        self.mass['too_light'] = fuzz.trimf(self.mass.universe, [70, 70, 110])
        self.mass['light'] = fuzz.trimf(self.mass.universe, [90, 110, 130])
        self.mass['normal'] = fuzz.trimf(self.mass.universe, [115, 135, 155])
        self.mass['heavy'] = fuzz.trimf(self.mass.universe, [140, 160, 180])
        self.mass['too_heavy'] = fuzz.trimf(self.mass.universe, [160, 200, 200])

        self.ref_angular_vel['min'] = fuzz.trimf(self.ref_angular_vel.universe, [0.0, 0.0, 0.5])
        self.ref_angular_vel['normal'] = fuzz.trimf(self.ref_angular_vel.universe, [0.0, 0.5, 1])
        self.ref_angular_vel['max'] = fuzz.trimf(self.ref_angular_vel.universe, [0.5, 1.0, 1.0])

        self.mass_scaling_factor['v_min'] = fuzz.trimf(self.mass_scaling_factor.universe, [0.2, 0.2, 0.4])
        self.mass_scaling_factor['min'] = fuzz.trimf(self.mass_scaling_factor.universe, [0.2, 0.4, 0.6])
        self.mass_scaling_factor['normal'] = fuzz.trimf(self.mass_scaling_factor.universe, [0.4, 0.6, 0.8])
        self.mass_scaling_factor['max'] = fuzz.trimf(self.mass_scaling_factor.universe, [0.6, 0.8, 1])
        self.mass_scaling_factor['v_max'] = fuzz.trimf(self.mass_scaling_factor.universe, [0.8, 1, 1])

        # Fuzzy rules
        self.rule1 = ctrl.Rule(self.mass['too_light']   &  self.ref_angular_vel['min']    ,self.mass_scaling_factor['v_max'])
        self.rule2 = ctrl.Rule(self.mass['too_light']   &  self.ref_angular_vel['normal'] ,self.mass_scaling_factor['max'])
        self.rule3 = ctrl.Rule(self.mass['too_light']   &  self.ref_angular_vel['max']    ,self.mass_scaling_factor['max'])

        self.rule4 = ctrl.Rule(self.mass['light']       &  self.ref_angular_vel['min']    ,self.mass_scaling_factor['max'])
        self.rule5 = ctrl.Rule(self.mass['light']       &  self.ref_angular_vel['normal'] ,self.mass_scaling_factor['max'])
        self.rule6 = ctrl.Rule(self.mass['light']       &  self.ref_angular_vel['max']    ,self.mass_scaling_factor['normal'])

        self.rule7 = ctrl.Rule(self.mass['normal']     &  self.ref_angular_vel['min']    ,self.mass_scaling_factor['max'])
        self.rule8 = ctrl.Rule(self.mass['normal']     &  self.ref_angular_vel['normal'] ,self.mass_scaling_factor['normal'])
        self.rule9 = ctrl.Rule(self.mass['normal']     &  self.ref_angular_vel['max']    ,self.mass_scaling_factor['normal'])

        self.rule10 = ctrl.Rule(self.mass['heavy']      &  self.ref_angular_vel['min']    ,self.mass_scaling_factor['normal'])
        self.rule11 = ctrl.Rule(self.mass['heavy']      &  self.ref_angular_vel['normal'] ,self.mass_scaling_factor['normal'])
        self.rule12 = ctrl.Rule(self.mass['heavy']      &  self.ref_angular_vel['max']    ,self.mass_scaling_factor['min'])

        self.rule13 = ctrl.Rule(self.mass['too_heavy']  &  self.ref_angular_vel['min']    ,self.mass_scaling_factor['normal'])
        self.rule14 = ctrl.Rule(self.mass['too_heavy']  &  self.ref_angular_vel['normal'] ,self.mass_scaling_factor['min'])
        self.rule15 = ctrl.Rule(self.mass['too_heavy']  &  self.ref_angular_vel['max']    ,self.mass_scaling_factor['v_min'])

        # Fuzzy control system and control system simulator
        self.mass_scale_factor_ctrl = ctrl.ControlSystem([self.rule1, self.rule2, self.rule3,self.rule4, self.rule5, self.rule6,  self.rule7, self.rule8,self.rule9,  self.rule10, self.rule11, self.rule12, self.rule13, self.rule14,self.rule15])
        self.mass_scale_factor_sim = ctrl.ControlSystemSimulation(self.mass_scale_factor_ctrl)
        print("--> Linear Velocity Planner Fuzzy-2 MSF Initiated!")
        # rospy.sleep(1)

    
    def set_mass(self, mass):
        self.mass_input = mass

    
    def run(self):
        """
        Apply inputs to inference system and compute output
        """
        self.mass_scale_factor_sim.input['mass'] = self.mass_input
        self.mass_scale_factor_sim.input['ref_angular_vel'] = self.ref_angular_vel_val
        self.mass_scale_factor_sim.compute()

        return self.mass_scale_factor_sim.output['mass_scaling_factor']

    
    def wref_callback(self, msg):
        self.ref_angular_vel_val = abs(msg.data) / self.ang_vel_max


    # def mass_callback(self, msg):
    #     self.mass_input = msg.data


    def sampleTimeSet(self):
        self.callback_time = rospy.get_time()


    def sampleTimeCalculate(self):
        self.sample_time = rospy.get_time() - self.last_callback_time
        self.last_callback_time = self.callback_time


    def distance_to_goal_callback(self, msg):
        self.distance_to_goal = msg.data


if __name__ == '__main__':
    
    rospy.init_node('fuzzy_mass_scaling_factor', anonymous=True)

    fuzzy2 = Fuzzy2(mass_total=80)

    rospy.Subscriber('/distance_to_goal', Float32, fuzzy2.distance_to_goal_callback)
    rospy.Subscriber('/angular_vel_output', Float32, fuzzy2.wref_callback)
    # rospy.Subscriber('/mass_input', Float32, fuzzy2.mass_callback)

    mass_scaler_fact_pub = rospy.Publisher('/mass_scaling_factor', Float32, queue_size=10)

    rate = rospy.Rate(50)
    
    flag = True
    while not rospy.is_shutdown():
        fuzzy2.sampleTimeSet()
        output = fuzzy2.run()

        # if fuzzy2.distance_to_goal > 1:
        #     mass_scaler_fact_pub.publish(output)
        # elif fuzzy2.distance_to_goal <= 1 and fuzzy2.distance_to_goal > 0.2:
        #     if flag:
        #         output_new = output
        #         flag = False

        #     if output_new > 0.2:
        #         output_new = output_new - fuzzy2.sample_time * 0.1
        #         mass_scaler_fact_pub.publish(output_new)
        #     else:
        #         mass_scaler_fact_pub.publish(output_new)

        # print('Distance to Goal :    {:.5f}'.format(fuzzy2.distance_to_goal))
        # print('MSF Out          :    {:.5f}'.format(output))
        # print('Sample Time      :    {:.5f}'.format(fuzzy2.sample_time))

        mass_scaler_fact_pub.publish(output)

        fuzzy2.sampleTimeCalculate()
        rate.sleep()