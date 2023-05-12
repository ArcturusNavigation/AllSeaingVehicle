# SIMULATOR FOR PID CONTROLLER
# based off of https://www.youtube.com/watch?v=ZMI_kpNUgJM
# will be used to tune gains, likely with Ziegler-Nichols method
# https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method

import numpy as np
import matplotlib.pyplot as plt
import turtle
import time

# GLOBAL PARAMS
TIMER = 0
TIME_STEP = 0.1
SETPOINT_W = 25
SETPOINT_V = 25
SIM_TIME = 200
MAX_THRUST = 100  # maximum T1 + T2 val, N
D = 20  # drag force, N
V0 = 10  # initial forward velocity, m/s
W0 = 0  # initial angular velocity, rad/s
m = 10  # mass of one hull, kg
I = 10  # Approximate boat moment of inertia (kg*m^2)
Xt = 1  # distance from thrusters to CoM of boat (m)
# ------------
# ---FORWARD VELOCITY PID GAINS---
KPV = 1
KIV = 0.0
KDV = 0.0
# ---ANGULAR VELOCITY PID GAINS---
KPW = 0.025
KIW = 0.001
KDW = 0.001
W_ERROR_BOUND = 0.05
# ---------------


class Simulation(object):
    def __init__(self):
        self.Insight = Boat()
        self.pidv = PID(KPV, KIV, KDV, SETPOINT_V)  # forward velocity pid controller
        self.pidw = PID(KPW, KIW, KDW, SETPOINT_W)  # angular velocity pid controller
        self.screen = turtle.Screen()
        self.screen.setup(800, 600)
        self.sim = True
        self.timer = 0
        self.times = np.array([])
        self.vs = np.array([])
        self.ws = np.array([])
        self.M = np.array([])
        self.R = np.array([])
        self.targetv = np.array([])
        self.targetw = np.array([])

    def cycle(self):
        M = 0
        R = 1
        while self.sim:
            M += self.pidv.compute(self.Insight.get_dy())  # compute new T1+T2
            if M > MAX_THRUST:
                print("M too large")
                M = MAX_THRUST
            if M < 0:
                M = 0
            print("M: " + str(M))
            self.Insight.set_ddy(M)
            difw = self.Insight.get_w() - SETPOINT_W
            if (
                abs(difw) > W_ERROR_BOUND
            ):  # stop updating angular velocity after getting within error bound
                R += self.pidw.compute(self.Insight.get_w())  # compute new T1/T2
                print("R: " + str(R))
                self.Insight.set_w(R, M)

            self.Insight.set_dy()
            self.Insight.update_boat()
            self.Insight.update_target()

            print(self.Insight.get_w())
            print("error: " + str(self.pidv.error))
            print("output: " + str(self.pidv.output))
            time.sleep(TIME_STEP)
            self.timer += 1
            if self.timer > SIM_TIME:
                print("SIM ENDED")
                self.sim = False
            self.times = np.append(self.times, self.timer)
            self.vs = np.append(self.vs, self.Insight.dy)
            self.ws = np.append(self.ws, self.Insight.w)
            self.M = np.append(self.M, M)
            self.R = np.append(self.R, R)
            self.targetv = np.append(self.targetv, SETPOINT_V)
            self.targetw = np.append(self.targetw, SETPOINT_W)
        graph(self.times, self.vs, self.ws, self.M, self.R, self.targetv, self.targetw)


def graph(x, y1, y2, y3, y4, y5, y6):
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True)
    # fig.suptitle('antiwindup')
    ax1.set(ylabel="forward \nvelocity \n(m/s)")
    ax1.plot(x, y1)
    ax1.plot(x, y5)
    ax2.set(ylabel="angular \nvelocity \n(rad/s)")
    ax2.plot(x, y2, "tab:red")
    ax2.plot(x, y6)
    ax3.set(ylabel="T1 + T2")
    ax3.plot(x, y3)
    ax4.set(ylabel="T1/T2")
    ax4.plot(x, y4)
    plt.show()


class Boat(object):
    def __init__(self):
        global Boat
        self.Boat = turtle.Turtle()
        boat_img = "boat.gif"
        turtle.register_shape(boat_img)
        # self.Boat.shape(boat_img)
        self.Boat.shape("square")
        self.Boat.color("orange")
        self.Boat.home()
        self.Boat.speed(0)
        self.target = turtle.Turtle()
        self.target.shape("circle")
        self.target.color("black")
        self.target.home()
        self.target.speed(0)
        # kinematics
        self.ddy = 0.0
        self.dy = V0
        self.w = W0
        self.tdy = SETPOINT_V
        self.tw = SETPOINT_W

    def set_ddy(self, M):
        self.ddy = (M - D) / (2 * m)

    def get_ddy(self):
        return self.ddy

    def set_dy(self):
        self.dy += self.ddy * TIME_STEP

    def get_dy(self):
        return self.dy

    def set_w(self, R, M):
        self.w = Xt * M * (R - 1 / R) / (2 * I)
        # self.w = 10

    def get_w(self):
        return self.w

    def update_boat(self):
        self.Boat.forward(self.dy * TIME_STEP)
        self.Boat.setheading(self.Boat.heading() + self.w * TIME_STEP)

    def update_target(self):
        self.target.forward(self.tdy * TIME_STEP)
        self.target.setheading(self.target.heading() + self.tw * TIME_STEP)


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
        self.output = (
            self.kp * self.error
            + self.ki * self.integral_error
            + self.kd * self.derivative_error
        )
        return self.output

    def get_kpe(self):
        return self.kp * self.error

    def get_kde(self):
        return self.kd * self.derivative_error

    def get_kie(self):
        return self.ki * self.integral_error


def main():
    sim = Simulation()
    sim.cycle()


main()
