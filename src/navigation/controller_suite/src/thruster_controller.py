#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from mavros_msgs.srv import CommandLong
from utility.constants import THR_BL, THR_BR, PWM_MIN, PWM_MAX, PWM_MID, IMG_WIDTH

class ThrusterController:

    K_P_ROT = 0.7
    FORWARD_PWM = 1600

    def __init__(self):

        rospy.init_node('thruster_controller', anonymous=True)

        # Current robot velocity
        self.center = Float64()
        self.curr_pwm_l = PWM_MID
        self.curr_pwm_r = PWM_MID

        # Subscribe to the command velocity topic
        rospy.Subscriber("/perception_suite/buoy_center", Float64, self.calc_pwm_values)

        rospy.spin()

    def send_pwm(self, channel, value):
        proxy = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        print(f"Sending pwm value {value} to channel {channel}")
        return proxy(
            command=183,
            param1=float(channel),
            param2=float(value)
        ).success

    def calc_pwm_values(self, center):

        # Proportional error
        rot_diff = IMG_WIDTH / 2 - center.data
        delta_pwm_rot = self.K_P_ROT * rot_diff

        # Add to pwm
        self.curr_pwm_l = self.FORWARD_PWM - delta_pwm_rot
        self.curr_pwm_r = self.FORWARD_PWM + delta_pwm_rot

        # Clip pwm value
        self.curr_pwm_l = max(min(self.curr_pwm_l, PWM_MAX), PWM_MIN)
        self.curr_pwm_r = max(min(self.curr_pwm_r, PWM_MAX), PWM_MIN)

        # Move robot based on calculated PWM values
        self.send_pwm(channel=THR_BL, value=self.curr_pwm_l)
        self.send_pwm(channel=THR_BR, value=self.curr_pwm_r)

if __name__ == '__main__':
    try:
        ThrusterController()
    except rospy.ROSInterruptException:
        pass
