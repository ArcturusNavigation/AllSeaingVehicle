# SIMULATOR FOR PID CONTROLLER
# based off of https://www.youtube.com/watch?v=ZMI_kpNUgJM
# will be used to tune gains, likely with Ziegler-Nichols method 
# https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method

import numpy as np
import matplotlib.pyplot as plt
import turtle 
import time

#GLOBAL PARAMS
TIMER = 0
TIME_STEP = 0.001
SETPOINT_W = 0
SETPOINT_V = 10
SIM_TIME = 100
V0 = 0 # initial forward velocity, m/s
W0 = 0 # initial angular velocity, rad/s
m = 1 # mass of one hull, kg
MAX_THRUST = 15 #Newtons
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
#---------------

class Simulation(object):
	def __init__(self):
		self.Insight = Boat()
		self.pidv = PID(KPV,KIV,KDV,SETPOINT_V) # forward velocity pid controller
		self.pidw = PID(KPW,KIW,KDW,SETPOINT_W) # angular velocity pid controller
		self.screen = turtle.Screen()
		self.screen.setup(800,600)
		self.marker = turtle.Turtle()
		self.marker.penup()
		self.marker.left(180)
		# self.marker.goto(15,SETPOINT)
		# self.marker.color('red')
		self.sim = True
		self.timer = 0
		self.poses = np.array([])
		self.times = np.array([])
		self.kpev = np.array([])
		self.kdev = np.array([])
		self.kiev = np.array([])
		self.kpew = np.array([])
		self.kdew = np.array([])
		self.kiew = np.array([])
		self.mag = np.array([])
		self.ratio = np.array([])
	def cycle(self):
		while(self.sim):
			M = self.pidv.compute(self.Insight.get_dy()) # compute new T1+T2
			print(M)
			R = self.pidw.compute(self.Insight.get_w()) # compute new T1/T2
			self.Insight.set_ddy(M)
			self.Insight.set_dy()
			self.Insight.set_w(R)
			self.Insight.set_theta()
			self.Insight.set_y()
			time.sleep(TIME_STEP)
			self.timer += 1
			if self.timer > SIM_TIME:
				print("SIM ENDED")
				self.sim = False
			elif self.Insight.get_y() > 700:
				print("OUT OF BOUNDS")
				self.sim = False
			elif self.Insight.get_y() < -700:
				print("OUT OF BOUNDS")
				self.sim = False
			self.poses = np.append(self.poses,self.Insight.get_y())
			self.times = np.append(self.times,self.timer)
			self.kpev = np.append(self.kpev,self.pidv.get_kpe())
			self.kdev = np.append(self.kdev,self.pidv.get_kde())
			self.kiev = np.append(self.kiev,self.pidv.get_kie())
			self.kpew = np.append(self.kpew,self.pid.get_kpe())
			self.kdew = np.append(self.kdew,self.pid.get_kde())
			self.kiew = np.append(self.kiew,self.pid.get_kie())
			self.mag = np.append(self.mag,M)
			self.ratio = np.append(self.ratio, R)
			
		graph(self.times,self.poses,self.kpev,self.kdev,self.kiev,self.mag)
		graph(self.times,self.poses,self.kpew,self.kdew,self.kiew,self.ratio)


def graph(x,y1,y2,y3,y4,y5):
	fig, (ax1, ax2,ax3,ax4,ax5) = plt.subplots(5, sharex=True)
	#fig.suptitle('antiwindup')
	ax1.set(ylabel='rocket \nHeight')
	ax1.plot(x,y1)
	ax2.set(ylabel='KP_error')
	ax2.plot(x,y2,'tab:red')
	ax3.set(ylabel='KD_error')
	ax3.plot(x,y3,'tab:orange')
	ax4.set(ylabel='KI_error')
	ax4.plot(x,y4,'tab:pink')
	ax5.set(ylabel='rocket \nThrust')
	ax5.plot(x,y5,'tab:brown')
	plt.show()
	
class Boat(object):
	def __init__(self):
		global Boat
		self.Boat = turtle.Turtle()
		self.Boat.shape('square')
		self.Boat.color('black')
		self.Boat.penup()
		self.Boat.home()
		self.Boat.speed(0)
		# kinematics
		self.ddy = 0
		self.dy = 0
		self.w = 0
	def set_ddy(self,M):
		self.ddy = M/(2*m)
	def get_ddy(self):
		return self.ddy
	def set_dy(self):
		self.dy += self.ddy * TIME_STEP
	def get_dy(self):
		return self.dy      
	def set_y(self):
		self.Boat.sety(self.y + self.dy * TIME_STEP)
	def get_y(self):
		self.y = self.Boat.ycor()
		return self.y
	def set_w(self, R, M): 
		self.w = Xt*M*(1/R+R)/(2*I)
	def get_w(self): 
		return self.w
	def set_theta(self): 
		self.Boat.setheading(self.theta + self.w * TIME_STEP)
	def get_theta(self): 
		self.y = self.Boat.heading()
		return self.y
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
	sim = Simulation()
	sim.cycle()
main()