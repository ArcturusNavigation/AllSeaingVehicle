import serial
import time
import numpy as np
import cv2
import math
import apriltag
import random as rng
# from pupil_apriltags import Detector

arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.1)
# arduino = serial.Serial(port='/dev/tty.usbserial-0264FEA5', baudrate=115200, timeout=.1)


STRAIGHT_VEL = 5
TURN_VEL = STRAIGHT_VEL/2
PICKUP_ANGLE = 40
DROPOFF_ANGLE = 120
ALPHA = 0.05
EPSILON_HEADING = 1
K_HEADING = 0.05
K_VEL_P = 8
K_CORR_P = 0.3
K_CORR_D = 0.008
Ki = 0.1

CAP = cv2.VideoCapture(0)
        
SCREEN_WIDTH = int(CAP.get(cv2.CAP_PROP_FRAME_WIDTH))
SCREEN_HEIGHT = int(CAP.get(cv2.CAP_PROP_FRAME_HEIGHT))
MIDPOINT = SCREEN_WIDTH // 2

P_CONTROL_BIAS = 0.25

FRAME_TIME = 5e-3


def main():
    car = Car()
    while [car.x0, car.y0, car.heading0] == [None, None, None]: # wait until readArduino receives usable data
        car.sendArduino()
        car.x0, car.y0, car.heading0 = car.readArduino()
    while True:
        if (time.time() - car.prev_time) > FRAME_TIME:
            car.prev_time = time.time()
            car.mega_counter += 1
            if car.mega_counter % 10 == 0:
                # print('MEGA' + str(car.mega_state))
                #car.cone_ret, car.cone_frame = CAP.read()
                car.april_ret, car.april_frame = CAP.read()

                if car.state == 2:
                    car.detect_april_tag(0.35)
                elif car.state == 7:
                    car.detect_april_tag(0.6)
                else:
                    # car.look_for_cone()
                    pass
            else:
                # print('MEGA' + str(car.mega_state))
                car.readArduino()
                # car.mega_state = 0
                # continue looping until readArduino receives usable data
                if [car.x_raw, car.y_raw, car.heading_raw] != [None, None, None]: 
                    
                    car.setXYH()

                    if car.cone_position:
                        if car.prev_state is None:
                            car.avoid_cone()
                            car.prev_state = car.state
                        car.state = 10
                    elif car.state == 10:
                        car.go()
                        if car.mini_state == 2:
                            car.state = car.prev_state
                            car.prev_state = None
                    if car.state == 0: ## go to AED waypoint #1
                        car.target_x = 1.5
                        car.target_y = 1.65
                        car.go()
                        if car.mini_state == 2:
                            car.mini_state = 0
                            car.state = 1
                    elif car.state == 1: # go to AED waypoint #2
                        car.target_x = 0.75
                        car.target_y = 1.65
                        car.go()
                        if car.mini_state == 2:
                            car.mini_state = 0
                            car.state = 2
                    elif car.state == 2: # go to AED 
                        if car.mini_state == 2: 
                            car.state = 3
                            car.mini_state = 0
                    elif car.state == 3: # pickup AED
                        car.stop()
                        car.pickupAED()
                        print('Success! AED picked up')
                        car.pickup_counter += 1 
                        if car.pickup_counter > 200:
                            car.mini_state = 0
                            car.state = 4
                    elif car.state == 4: # back up
                        car.back()
                        car.backup_counter += 1
                        if car.backup_counter > 200:
                            car.mini_state = 0
                            car.state = 5
                    elif car.state == 5: # turn around
                        car.target_x = 2
                        car.target_y = 1
                        car.go()
                        if car.mini_state == 2: 
                            car.mini_state = 0
                            car.state = 7
                            # car.stop()
                    # elif car.state == 6: # go forward
                    #     car.target_x = 2.75
                    #     car.target_y = 1
                    #     car.go()
                    #     if car.mini_state == 2: 
                    #         car.mini_state = 0
                    #         car.state = 7
                    #         # car.stop()
                    elif car.state == 7: # go to april tag
                        if car.mini_state == 2: 
                            car.mini_state = 0
                            car.state = 8
                    elif car.state == 8: # dropoff aed
                        car.stop()
                        car.dropoffAED()
                        print('Success! AED dropped off')
                    
                car.filter()
                car.sendArduino()
                car.printCurr()
                # print('state' + str(car.state))
                
class Car(object): 
    def __init__(self): 
        self.detector = apriltag.Detector()
        self.cone_ret = None
        self.cone_frame = None
        self.april_ret = None
        self.april_frame = None
        self.mega_counter = 0

        self.epsilon_dist = 0.1
        
        self.target_x = 1
        self.target_y = 1.65
        self.target_heading = 0

        self.leftVel = 0
        self.rightVel = 0
        self.servoAngle = 90

        self.x0 = None
        self.y0 = None
        self.heading0 = None

        self.x = 0
        self.y = 0
        self.heading = 0

        self.x_raw = 0
        self.y_raw = 0
        self.heading_raw = 0

        self.filtLeftVel = 0
        self.filtRightVel = 0
        self.filtServoAngle = 90

        self.mega_state = 0
        self.state = 0
        self.prev_time = time.time()
        self.april_time = time.time()
        self.prev_state = None

        self.pickup_counter = 0
        self.backup_counter = 0
        self.dropoff_counter = 0

        self.mini_state = 0
        self.cone_position = None
        self.april_tag = None
        self.tagangle = 0

        self.intrisic = [640,640,960,540]
        self.tagsize = 0.100  #physical size of printed tag, unit = meter
        self.threshold = 14  # tolerable yaw
        self.vote_array = []

        self.sumerror = 0
        self.prev_error_heading = 0

    def readArduino(self):
        '''
        Read output from Arduino: x, y, heading.
        Returns [None, None, None] if response is not in the right format
        '''
        self.x_raw, self.y_raw, self.heading_raw = [None, None, None]
        if arduino.in_waiting > 0:
            try:
                response = arduino.readline().decode().strip().split(',')
                if len(response) == 3 and all(len(i) > 0 for i in response):
                    response = [float(i) for i in response]
                    self.x_raw, self.y_raw, self.heading_raw = response
            except:
                pass
        return self.x_raw, self.y_raw, self.heading_raw
    

    def sendArduino(self): 
        print('left vel: %f, right vel %f' % (self.filtLeftVel, self.filtRightVel))
        msg = f"{self.filtLeftVel},{self.filtRightVel},{self.filtServoAngle}\n".encode() # encode message as bytes
        arduino.write(msg)
 

if __name__ == "__main__":
    main()
