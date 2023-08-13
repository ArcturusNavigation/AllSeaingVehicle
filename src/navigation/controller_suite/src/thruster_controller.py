#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandLong
from utility.constants import THR_BL, THR_BR, PWM_MIN, PWM_MAX, PWM_MID

class ThrusterController:

    K_P_ROT = 5
    K_P_LIN = 5

    def __init__(self):

        rospy.init_node('thruster_controller', anonymous=True)

        # Current robot velocity
        self.vel = Twist()
        self.curr_pwm_l = PWM_MID
        self.curr_pwm_r = PWM_MID

        # Subscribe to the command velocity topic
        rospy.Subscriber("/cmd_vel", Twist, self.calc_pwm_values)
        rospy.Subscriber("/perception_suite/zed_velocity", Twist, self.get_velocity)

        rospy.spin()

    def send_pwm(self, channel, value):
        proxy = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        print(f"Sending pwm value {value} to channel {channel}")
        return proxy(
            command=183,
            param1=float(channel),
            param2=float(value)
        ).success

    def get_velocity(self, vel):
        self.vel = vel

    def calc_pwm_values(self, cmd_vel):

        # Proportional error
        delta_pwm_lin = self.K_P_LIN * (cmd_vel.linear.x - self.vel.linear.x)
        delta_pwm_rot = self.K_P_ROT * (cmd_vel.angular.z - self.vel.angular.z)

        # Add to pwm
        self.curr_pwm_l += delta_pwm_lin + delta_pwm_rot
        self.curr_pwm_r += delta_pwm_lin - delta_pwm_rot

        # Clip pwm value
        self.curr_pwm_l = max(min(self.curr_pwm_l, PWM_MAX), PWM_MIN)
        self.curr_pwm_r = max(min(self.curr_pwm_r, PWM_MAX), PWM_MIN)
        
        print(self.curr_pwm_l)
        print(self.curr_pwm_r)

        # Move robot based on calculated PWM values
        #self.send_pwm(channel=THR_BL, value=self.curr_pwm_l)
        #self.send_pwm(channel=THR_BR, value=self.curr_pwm_r)

if __name__ == '__main__':
    try:
        ThrusterController()
    except rospy.ROSInterruptException:
        pass
