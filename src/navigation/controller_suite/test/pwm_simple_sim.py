#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import pygame
import math

class BoatSimulator:
    def __init__(self):
        # ROS Node Initialization
        rospy.init_node('boat_simulator', anonymous=True)
        rospy.Subscriber("/controller/commanded_pwm", Float64MultiArray, self.pwm_callback)
        
        # Pygame Initialization
        pygame.init()
        self.width, self.height = 800, 600
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption('Boat Simulator')

        # Boat Initialization
        self.boat_pos = [self.width / 2, self.height / 2]
        self.boat_angle = 0 # In degrees
        self.boat_speed = [0, 0]
        self.drag_factor = 0.02 # Simple drag effect

        # Run the simulation
        self.run()

    def pwm_callback(self, msg):
        # Convert PWM values to force (assuming linear relation)
        left_thruster_force = msg.data[0] / 100.0
        right_thruster_force = msg.data[1] / 100.0

        # Calculate the resultant force and torque
        force = (left_thruster_force + right_thruster_force) / 2
        torque = (right_thruster_force - left_thruster_force) / 10.0 # Adjust denominator for desired turning sensitivity

        # Update boat's speed and angle
        self.boat_speed[1] -= force * math.cos(math.radians(self.boat_angle)) # Changing x component to control up and down
        self.boat_speed[0] += force * math.sin(math.radians(self.boat_angle)) # Changing y component to control left and right
        self.boat_angle += torque

    def run(self):
        running = True
        clock = pygame.time.Clock()

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Update the boat's position
            self.boat_pos[0] += self.boat_speed[0]
            self.boat_pos[1] += self.boat_speed[1]

            # Apply drag
            self.boat_speed[0] -= self.boat_speed[0] * self.drag_factor
            self.boat_speed[1] -= self.boat_speed[1] * self.drag_factor

            # Draw the scene
            self.screen.fill((135, 206, 250)) # Sky blue background

            # Draw the boat (rotating the polygon)
            pygame.draw.polygon(self.screen, (169, 169, 169), self.rotate_boat())

            pygame.display.flip()
            clock.tick(60)

        pygame.quit()

    def rotate_boat(self):
        # Function to get the rotated polygon for the boat
        angle_rad = math.radians(self.boat_angle)
        rotated_polygon = []
        boat_polygon = [
            (0, -20),
            (-10, 20),
            (10, 20)
        ]
        for x, y in boat_polygon:
            x_rot = x * math.cos(angle_rad) - y * math.sin(angle_rad) + self.boat_pos[0]
            y_rot = x * math.sin(angle_rad) + y * math.cos(angle_rad) + self.boat_pos[1]
            rotated_polygon.append((x_rot, y_rot))
        return rotated_polygon

if __name__ == '__main__':
    try:
        BoatSimulator()
    except rospy.ROSInterruptException:
        pass
