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
	def cycle(self): 
		M = 0
		R = 1
		while(True): # change to a condition for stopping controller
			M += self.pidv.compute(self.Insight.get_dy()) # compute new T1+T2
			if M > MAX_THRUST: 
				M = MAX_THRUST
			if M < 0: 
				M = 0
			difw = self.Insight.get_w() - SETPOINT_W	
			if abs(difw) > W_ERROR_BOUND: 
				R += self.pidw.compute(self.Insight.get_w()) # compute new T1/T2
			self.Insight.thrust(R,M) # send thrust command

class Boat(object):
	def __init__(self):
		self.dy = 0
		self.w = 0
	def get_dy(self): 
		return self.dy ## find current forward velocity
	def get_w(self): 
		return self.w ## find current angular velocity 
	def thrust(R, M): 
		return ## send command to thrusters 
		
	
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

def main():
	c = Controller()
	c.cycle()
main()