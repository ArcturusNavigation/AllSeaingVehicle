import rospy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import OverrideRCIn

# PID CONTROLLER 
# INPUT: TARGET FORWARD(M/S) AND ANGULAR VELOCITY(RAD/S)
# OUTPUT: TWO THRUSTER FORCE VALUES (N)
import numpy as np

#GLOBAL PARAMS
TIME_STEP = 0.001
SETPOINT_W = 0
SETPOINT_V = 10
V0 = 0 # initial forward velocity, m/s
W0 = 0 # initial angular velocity, rad/s
m = 1 # mass of one hull, kg
MAX_THRUST = 100 #Newtons

MIN_PWM = 1100
ZERO_PWM = 1500
MAX_PWM = 1900	
I = 1 # Approximate boat moment of inertia
Xt = 1 # distance from thrusters to CoM of boat
#------------
#---FORWARD VELOCITY PID GAINS--- 
KPV = 0.6
KIV = 0.0
KDV= 0.0
#---ANGULAR VELOCITY PID GAINS--- 
KPW = 0.6
KIW = 0.0
KDW = 0.0
W_ERROR_BOUND = 0.05
#---------------

class Controller(object): 
	def __init__(self): 
		self.Insight = Boat()
		self.pidv = PID(KPV, KIV, KDV, SETPOINT_V)
		self.pidw = PID(KPW, KIW, KDW, SETPOINT_W)
		self.M = 0
		self.R = 1
		
	def cycle(self): 
		self.M += self.pidv.compute(self.Insight.get_dy()) # compute new T1+T2
		if self.M > MAX_THRUST: 
			self.M = MAX_THRUST
		if self.M < 0: 
			self.M = 0
		difw = self.Insight.get_w() - SETPOINT_W	
		if abs(difw) > W_ERROR_BOUND: 
			self.R += self.pidw.compute(self.Insight.get_w()) # compute new T1/T2
		self.Insight.thrust(self.R,self.M) # send thrust command

class Boat(object):
	def __init__(self):
		self.dy = 0
		self.w = 0
	def get_dy(self): 
		return self.dy ## find current forward velocity
	def get_w(self): 
		return self.w ## find current angular velocity 
	def PWM(R, M): 
		pass ## implement toya node, send PWM to thrusters
		
	
class PID(object):
	def __init__(self,KP,KI,KD,target):
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
		self.output = self.kp*self.error + self.ki*self.integral_error + self.kd*self.derivative_error
		if self.output >= MAX_THRUST:
			self.output = MAX_THRUST
		elif self.output <= 0:
			self.output = 0
		return self.output
		
	def get_kpe(self):
		return self.kp*self.error
	def get_kde(self):
		return self.kd*self.derivative_error
	def get_kie(self):
		return self.ki*self.integral_error


def velocity_callback(data):
    # Process the velocity data received from the Pixhawk
    linear_velocity_x = data.twist.linear.x
    linear_velocity_y = data.twist.linear.y
    linear_velocity_z = data.twist.linear.z
    angular_velocity_x = data.twist.angular.x
    angular_velocity_y = data.twist.angular.y
    angular_velocity_z = data.twist.angular.z

    # Implement your control algorithm here
    # For example, let's assume a simple proportional controller for the linear velocity along the x-axis
    target_velocity = 1.0  # Desired linear velocity
    k_p = 0.5  # Proportional gain

    # Calculate the control command
    error = target_velocity - linear_velocity_x
    control_command = k_p * error

    # Convert the control command to PWM values


    # Create a PWM command message
    pwm_msg = OverrideRCIn()
    pwm_msg.channels = [left, right]  # Initialize all channels to zero

    # Set the appropriate channel value for your actuators
    # ... Modify the channel indices and values based on your actuators and requirements

    # Publish the PWM command
    pwm_pub.publish(pwm_msg)

if __name__ == '__main__':
    # c = Controller()
	# c.cycle()
    rospy.init_node('velocity_controller')
    rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, velocity_callback)
    pwm_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
    rospy.spin()