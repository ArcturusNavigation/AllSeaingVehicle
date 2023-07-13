import rospy
import numpy as np

class Buoy:
    def __init__(self, x,y, size,color = None):
        self.x = x
        self.y = y
        self.size = size
        self.color = color 
    def check_collision(self, other):
        if self.x - self.size < other.x + other.size and self.x + self.size > other.x - other.size:
            if self.y - self.size < other.y + other.size and self.y + self.size > other.y - other.size:
                return True #collision
        return False
    
    def check_path_collision(self, path):
        pass #TODO

    def draw(self):
        pass #TODO

class Vehicle:
    def __init__(self, x, y, angle, size):
        self.x = x
        self.y = y
        self.angle = angle
        self.size = size
        self.speed = 0
    def draw(self):
        pass #TODO

class SimpleSim:
    def __init__(self,objects = None, dt = 0.1):
        if objects is None:
            self.generate_random_map()
        self.vehicle = Vehicle(0,0,0,2)
        self.dt = dt 
        self.sub = rospy.Subscriber('/vehicle/cmd_vel', Twist, self.update)

    def generate_random_map(self):
        self.objects = [Buoy(),Buoy(),Buoy(),Buoy()]
    
    def update(self):
        pass #TODO

def main():



if __name__ == '__main__':
    main()