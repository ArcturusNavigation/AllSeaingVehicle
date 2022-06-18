#!/usr/bin/env python

import rospy
import numpy as np
import skimage
from geom_helper import quaternion_from_angle
import pygame

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from arcturus_pilot.msg import ProcessedWaypoint, WaypointReached
from sensor_suite.msg import Object, ObjectArray

ACCEPTANCE_RADIUS = 0.2
OCCUPANCY_RESOLUTION = 0.5
MAP_WIDTH = 70
MAP_HEIGHT = 55

CAMERA_DIST = 30

BUOY_RADIUS = 2.0 # m (inflated radius)
STEP_SIZE = 3
STEP_SIZE_HEADING = np.pi / 6

OCCUPANCY_WIDTH = int(MAP_WIDTH / OCCUPANCY_RESOLUTION)
OCCUPANCY_HEIGHT = int(MAP_HEIGHT / OCCUPANCY_RESOLUTION)

def convert(x):
    return int(round(x / OCCUPANCY_RESOLUTION))

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

def get_screen_coords(x, y):
    return int(x / MAP_WIDTH * SCREEN_WIDTH), int(SCREEN_HEIGHT - y / MAP_WIDTH * SCREEN_HEIGHT)

class WaypointSim():
    def __init__(self):
        self.objects = [

        ]

        self.boat_pos = np.array([0, 0], dtype=np.float32)
        self.boat_heading = 0
        self.other_obstacles = []

        self.waypoint_sub = rospy.Subscriber('arcturus_pilot/processed_waypoint', ProcessedWaypoint, self.waypoint_callback)
        self.pose_pub = rospy.Publisher('arcuturus_pilot/pose', PoseStamped, queue_size=10)
        self.objects_pub = rospy.Publisher('sensor_suite/objects', ObjectArray, queue_size=5)
        self.occupancy_pub = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=5)
        self.waypoint_reached_pub = rospy.Publisher('arcuturus_pilot/waypoint_reached', WaypointReached, queue_size=10)

        self.waypoints = []
        self.temp_waypoint = None
        self.current_order = 0

        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.step()
            self.draw()
            self.send_info()
            while True:
                key_pressed = False
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        key_pressed = True
                        break
                if key_pressed:
                    break

    def draw(self):
        self.screen.fill((255, 255, 255))
        pygame.draw.circle(self.screen, (0, 0, 255), get_screen_coords(self.boat_pos[0], self.boat_pos[1]), 20)

        waypoint = self.get_curr_waypoint()
        if waypoint != None:
            pygame.draw.circle(self.screen, (255, 0, 255), get_screen_coords(waypoint[0], waypoint[1]), 5)
        for object in self.objects:
            x, y = object[1], object[2]
            pygame.draw.circle(self.screen, (255, 0, 0), get_screen_coords(x, y), 10)

        for obstacle in self.other_obstacles:
            converted_obstacle = [get_screen_coords(x, y) for (x, y) in obstacle]
            pygame.draw.polygon(self.screen, (255, 0, 0), converted_obstacle)

    def waypoint_callback(self, waypoint):
        if waypoint.is_temp:
            self.temp_waypoint = (waypoint.x, waypoint.y, waypoint.heading)
            return
            
        while len(self.waypoints) <= waypoint.order:
            self.waypoints.append(())
        self.waypoints[waypoint.order] = (waypoint.x, waypoint.y, waypoint.heading)
        
    def get_curr_waypoint(self):
        if self.temp_waypoint is not None:
            return self.temp_waypoint
        if len(self.waypoints) <= self.current_order:
            return None
        return self.waypoints[self.current_order]

    def step(self):
        waypoint = self.get_curr_waypoint()
        if waypoint == None:
            rospy.logerr("no waypoint set")
            return
        else:
            waypoint_x, waypoint_y, waypoint_heading = waypoint[0], waypoint[1], waypoint[2]
            rospy.loginfo("setting setpoint: " + str(waypoint_x) + " " + str(waypoint_y) + " " + str(waypoint_heading))

            pos_step = np.array([waypoint_x, waypoint_y]) - self.boat_pos
            if np.linalg.norm(pos_step) > STEP_SIZE:
                pos_step = (pos_step / np.linalg.norm(pos_step)) * STEP_SIZE
            self.boat_pos += pos_step

            heading_step = waypoint_heading - self.boat_heading
            while heading_step >= np.pi:
                heading_step -= np.pi
            while heading_step <= -np.pi:
                heading_step += np.pi

            if abs(heading_step) > STEP_SIZE_HEADING:
                heading_step = STEP_SIZE_HEADING * heading_step / abs(heading_step)
            self.boat_heading += heading_step
            
            dist_sq = (self.boat_pos[0] - waypoint_x) ** 2 + (self.boat_pos[1] - waypoint_y) ** 2
            if dist_sq < ACCEPTANCE_RADIUS ** 2:
                self.boat_heading = waypoint_heading
                if self.temp_waypoint != None:
                    self.temp_waypoint = None
                else:
                    self.waypoint_reached_pub.publish(WaypointReached(self.current_order))
                    self.current_order += 1

    def send_objects(self):
        object_list = []
        # TODO: implement the FOV
        for object in self.objects:
            if np.linalg.norm(np.array([object[1], object[2]]) - self.boat_pos) > CAMERA_DIST:
                continue

            object_ros = Object()
            object_ros.label = object[0][0]
            object_ros.task_label = object[0][1]
            object_ros.pos = Point(object[1], object[2], 0)

            object_list.append(object_ros)
        
        object_list_ros = ObjectArray()
        object_list_ros.objects = object_list

        self.objects_pub.publish(object_list_ros)
    
    def send_occupancy(self):
        occupancy_ros = OccupancyGrid()
        occupancy_ros.header.stamp = rospy.Time.now()
        occupancy_ros.info.resolution = OCCUPANCY_RESOLUTION
        occupancy_ros.info.width = OCCUPANCY_WIDTH
        occupancy_ros.info.height = OCCUPANCY_HEIGHT

        grid = np.zeros((OCCUPANCY_HEIGHT, OCCUPANCY_WIDTH), dtype=np.int8)

        for object in self.objects:
            object_data = skimage.draw.ellipse(convert(object[1]), convert(object[0]), 
                BUOY_RADIUS, BUOY_RADIUS, shape=(OCCUPANCY_HEIGHT, OCCUPANCY_WIDTH))
            grid[object_data] = 100

        for obstacle in self.other_obstacles:
            obstacle_data = skimage.draw.polygon([convert(y) for (x, y) in obstacle], [convert(x) for (x, y) in obstacle], 
                (OCCUPANCY_HEIGHT, OCCUPANCY_WIDTH))
            grid[obstacle_data] = 100

        occupancy_ros.data = np.ndarray.tolist(np.ndarray.flatten(grid))

    def send_info(self):
        self.send_objects()
        self.send_occupancy()
        
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position = Point(x = self.boat_pos[0], y = self.boat_pos[1], z = 0)
        pose.pose.orientation = quaternion_from_angle(self.boat_heading)

        self.pose_pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('waypoint_sim')
    waypoint_sim = WaypointSim()
    waypoint_sim.run()
