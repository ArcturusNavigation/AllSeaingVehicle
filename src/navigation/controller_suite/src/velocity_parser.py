#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class PWMConverter:
    # Constants: Set these values according to your robot's requirements
    K_P = 10 # Proportional gain for the linear velocity control
    b = 5.0 # Offset for linear velocity control
    PWM_TURN = 500 # PWM value for turning
    PWM_MIN = 10 # Minimum PWM value to activate motors
    DRIFT_MULTIPLIER = 1.0 # Multiplier to correct for drift when moving straight

    def __init__(self):
        rospy.init_node('pwm_converter', anonymous=True)
        # Subscribe to the command velocity topic
        rospy.Subscriber("/cmd_vel", Twist, self.calc_pwm_values)
        # Publisher for the commanded PWM values
        self.pwm_pub = rospy.Publisher("/controller/commanded_pwm", Float64MultiArray, queue_size=10)
        
        # Variables to keep track of the system state
        self.lastCmdVelReceived = rospy.get_time()
        self.velLeftWheel = 0.0 # You may want to update these based on actual wheel velocities
        self.velRightWheel = 0.0 
        self.prevDiff = 0.0 # Previous differences in wheel velocities
        self.prevPrevDiff = 0.0 # Previous to previous differences in wheel velocities

        rospy.spin()

    def calc_pwm_values(self, cmdVel):
        # Update timestamp of the last command received
        self.lastCmdVelReceived = rospy.get_time()

        # Calculate the required PWM values for both wheels based on the desired linear velocity
        pwmLeftReq = self.K_P * cmdVel.linear.x + self.b
        pwmRightReq = self.K_P * cmdVel.linear.x + self.b

        # Check for turning command
        if cmdVel.angular.z != 0.0:
            if cmdVel.angular.z > 0.0: # Turn left
                pwmLeftReq = -self.PWM_TURN
                pwmRightReq = self.PWM_TURN
            else: # Turn right
                pwmLeftReq = self.PWM_TURN
                pwmRightReq = -self.PWM_TURN
        else: # Go straight
            # Calculate current difference in wheel velocities
            currDifference = self.velLeftWheel - self.velRightWheel
            avgDifference = (self.prevDiff + self.prevPrevDiff + currDifference) / 3
            self.prevPrevDiff = self.prevDiff
            self.prevDiff = currDifference

            # Correct PWM values to make the vehicle go straight
            pwmLeftReq -= int(avgDifference * self.DRIFT_MULTIPLIER)
            pwmRightReq += int(avgDifference * self.DRIFT_MULTIPLIER)

        # Handle low PWM values (below threshold)
        if abs(pwmLeftReq) < self.PWM_MIN:
            pwmLeftReq = 0
        if abs(pwmRightReq) < self.PWM_MIN:
            pwmRightReq = 0

        # Publish the calculated PWM values
        commanded_pwm = Float64MultiArray(data=[pwmLeftReq, pwmRightReq])
        self.pwm_pub.publish(commanded_pwm)

if __name__ == '__main__':
    try:
        PWMConverter()
    except rospy.ROSInterruptException:
        pass
