#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from mavros_msgs.srv import CommandLong
from utility.constants import THR_BL, THR_BR, PWM_MIN, PWM_MAX, PWM_MID, PWM_MAX_TURN

class DistanceAngleController:
    # Control constants
    K_P_ANGLE = 0.5
    CONSTANT_FORWARD_PWM = 1600
    REDUCE_DISTANCE_START = 5.0  # Start reducing speed when within this distance
    DISTANCE_THRESHOLD = 2  # in meters
    

    def __init__(self):
        rospy.init_node('distance_angle_controller', anonymous=True)

        self.curr_pwm_l = PWM_MID
        self.curr_pwm_r = PWM_MID

        # Subscribe to the result topic from the previous node
        rospy.Subscriber('/controller/waypoint_position', Float64MultiArray, self.calc_pwm_values)

        rospy.spin()

    def send_pwm(self, channel, value):
        proxy = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        rospy.loginfo(f"Sending pwm value {value} to channel {channel}")
        return proxy(
            command=183,
            param1=float(channel),
            param2=float(value)
        ).success

    def calc_pwm_values(self, data):
        # Extract data
        change_in_angle, distance = data.data

        print("RUNNING CONTROLLER")

        # If distance is less than threshold, set PWM to MID and return
        if distance < self.DISTANCE_THRESHOLD:
            start = rospy.Time.now()
            while rospy.Time.now() - start < rospy.Duration(2):
                self.curr_pwm_l = PWM_MID - 100
                self.curr_pwm_r = PWM_MID + 100
                self.send_pwm(channel=THR_BL, value=self.curr_pwm_l)
                self.send_pwm(channel=THR_BR, value=self.curr_pwm_r)
            
            print("WAYPOINT REACHED")
            return

        # Adjust forward PWM based on distance
        if distance <= self.REDUCE_DISTANCE_START:
            proportion = (distance - self.DISTANCE_THRESHOLD) / (self.REDUCE_DISTANCE_START - self.DISTANCE_THRESHOLD)
            forward_pwm = PWM_MID + (self.CONSTANT_FORWARD_PWM - PWM_MID) * proportion
        else:
            forward_pwm = self.CONSTANT_FORWARD_PWM

        # Proportional rotation based on angle difference
        if(change_in_angle > 70 or change_in_angle < -70):
            delta_pwm_rot = PWM_MAX_TURN

        if abs(change_in_angle) <= 5:
            delta_pwm_rot = 0
        elif abs(change_in_angle) >= 70:
            delta_pwm_rot = 200 if change_in_angle > 0 else -200
        else:
            # Scale the angle proportionally between the limits
            # For positive angles
            if change_in_angle > 0:
                delta_pwm_rot = ((change_in_angle - 5) / 65) * 200
            # For negative angles
            else:
                delta_pwm_rot = ((change_in_angle + 5) / 65) * 200
    
        # Calculate individual PWM for left and right thrusters
        self.curr_pwm_l = forward_pwm - delta_pwm_rot
        self.curr_pwm_r = forward_pwm + delta_pwm_rot

        # Clip pwm values to make sure they are within min and max
        self.curr_pwm_l = max(min(self.curr_pwm_l, PWM_MAX), PWM_MIN)
        self.curr_pwm_r = max(min(self.curr_pwm_r, PWM_MAX), PWM_MIN)

        # Send the PWM values to the thrusters
        self.send_pwm(channel=THR_BL, value=self.curr_pwm_l)
        self.send_pwm(channel=THR_BR, value=self.curr_pwm_r)

if __name__ == '__main__':
    try:
        DistanceAngleController()
    except rospy.ROSInterruptException:
        pass
