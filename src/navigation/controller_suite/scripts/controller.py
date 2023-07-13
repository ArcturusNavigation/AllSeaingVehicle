#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist

# PID CONTROLLER 
# INPUT: TARGET FORWARD(M/S) AND ANGULAR VELOCITY(RAD/S)
# OUTPUT: TWO THRUSTER FORCE VALUES (N)
import numpy as np

# GLOBAL PARAMS
TIME_STEP = 0.001
MIN_PWM = 1100
ZERO_PWM = 1500
MAX_PWM = 1900	
# FORWARD VELOCITY PID GAINS
KPV = 0.6
KIV = 0.0
KDV= 0.0
# ANGULAR VELOCITY PID GAINS
KPW = 0.6
KIW = 0.0
KDW = 0.0
W_ERROR_BOUND = 0.05
#---------------

class VelocityController(object): 
	def __init__(self): 
		self.pidv = None
		self.pidw = None
		self.feedback_linear = 0
		self.feedback_angular = 0
		self.commanded_linear = 0
		self.commanded_angular = 0
		self.rate = rospy.Rate(2)
		# self.sub1 = rospy.Subscriber('/mavros/velocity_unstamped', Twist, c.stateCallback) #We subscribe to the boats velocity
		# self.sub2 = rospy.Subscriber('/mavros', Twist, c.commandCallback) #We subscribe to the boats velocity
		self.pub = rospy.Publisher('/controller/output_pwm', Twist, queue_size=2)
		## send PWM command
	def stateCallback(msg, self): 
		# Controller takes in boat data from msg that it subscribes to
		rospy.loginfo(msg.data)#prints on terminal
		self.feedback_linear = msg.twist.linear.x
		self.feedback_angular = msg.twist.angular.z

	def callback2(msg, self):
		rospy.loginfo(msg.data)
		self.commanded_linear = msg.twist.linear.x
		self.commanded_angular = msg.twist.angular.z
		if not self.pidv: 
			self.pidv = PID(KPV, KIV, KDV, self.commanded_linear)
		if not self.pidw: 
			self.pidw = PID(KPW, KIW, KDW, self.commanded_linear)
		M = self.pidv.compute(self.Insight.get_dy()) # compute new PWM1+PWM2
		difw = self.feedback_angular - self.commanded_angular
		if abs(difw) > W_ERROR_BOUND: # stop changing magnitude when angular velocity within bound
			R = self.pidw.compute(self.feedback_angular) # compute new PWM1/PWM2
		PWM1 = (M * R) / (R + 1)
		PWM2 = M / (R + 1)
		self.pub.publish([self.bounded(PWM1), self.bounded(PWM2)])

	def bounded(PWM, self): 
		if PWM< MIN_PWM: 
			return MIN_PWM
		if PWM > MAX_PWM: 
			return MAX_PWM

class PID(object):

	def __init__(self, KP, KI, KD, target):
		self.kp = KP
		self.ki = KI
		self.kd = KD 
		self.setpoint = target
		self.error = 0
		self.integral_error = 0
		self.error_last = 0
		self.derivative_error = 0
		self.output = 0

	def compute(self, curr):
		self.error = self.setpoint - curr
		self.integral_error += self.error * TIME_STEP
		self.derivative_error = (self.error - self.error_last) / TIME_STEP
		self.error_last = self.error
		self.output = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error
		return self.output

	def get_kpe(self):
		return self.kp * self.error

	def get_kde(self):
		return self.kd * self.derivative_error

	def get_kie(self):
		return self.ki * self.integral_error
  

def __main__():
	rospy.init_node('controller_node')
	c = VelocityController()

	rospy.spin()
