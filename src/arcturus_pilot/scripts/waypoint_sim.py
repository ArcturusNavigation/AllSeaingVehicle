#!/usr/bin/env python

import rospy
import numpy as np
import skimage
from geom_helper import quaternion_from_angle
import pygame
import random

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from arcturus_pilot.msg import ProcessedWaypoint, WaypointReached, SkipWaypoint
from sensor_suite.msg import Object, ObjectArray

ACCEPTANCE_RADIUS = 0.2
OCCUPANCY_RESOLUTION = 0.1
MAP_WIDTH = 70
MAP_HEIGHT = 55

CAMERA_DIST = 30

BUOY_RADIUS = 0.5 # m (inflated radius)
STEP_SIZE = 1
STEP_SIZE_HEADING = np.pi / 6

OCCUPANCY_WIDTH = int(MAP_WIDTH / OCCUPANCY_RESOLUTION)
OCCUPANCY_HEIGHT = int(MAP_HEIGHT / OCCUPANCY_RESOLUTION)

def convert(x):
    return int(round(x / OCCUPANCY_RESOLUTION))

def convert_image_coords(x, y):
    return x / 5, 55 - y / 5

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

def get_screen_coords(x, y):
    return int((x / MAP_WIDTH) * SCREEN_WIDTH), int(SCREEN_HEIGHT - (y / MAP_WIDTH) * SCREEN_HEIGHT)

class WaypointSim():
    def __init__(self):
        self.objects = [
            ((0, 1), 6.0, 16.0),
            ((1, 1), 10.0, 16.0),
            ((0, 1), 6.0, 34.0),
            ((1, 1), 10.0, 34.0),

            ((0, 2), 7.0, 44.0),
            ((1, 2), 11.0, 43.0),
            ((0, 2), 11.0, 49.0),
            ((1, 2), 12.0, 46.0),
            ((0, 2), 16.0, 50.0),
            ((1, 2), 16.0, 46.0),
            ((0, 2), 20.0, 48.0),
            ((1, 2), 20.0, 44.0),
            ((0, 2), 24.0, 47.0),
            ((1, 2), 24.0, 43.0),
            ((0, 2), 28.0, 47.0),
            ((1, 2), 28.0, 43.0),
            ((0, 2), 34.0, 48.0),
            ((1, 2), 34.0, 44.0),
            ((0, 2), 39.0, 47.0),
            ((1, 2), 38.0, 43.0),
            ((0, 2), 44.0, 43.0),
            ((1, 2), 40.0, 40.0),
            ((0, 2), 46.0, 39.0),
            ((1, 2), 42.0, 37.0),

            ((2, 2), 10.0, 48.0),
            ((2, 2), 24.0, 44.0),
            ((2, 2), 38.0, 45.0),
            ((2, 2), 42.0, 39.0),

            ((0, 4), 38.0, 26.0),
            ((1, 4), 38.0, 31.0),
            ((3, 4), 23.0, 29.0),

            ((5, 3), 56.0, 31.0),
            ((6, 6), 49.0, 8.0),
            ((7, 5), 32.0, 14.0),

            ((4, 7), 17.0, 14.0),
            ((4, 7), 17.0, 7.0),
        ]
        random.shuffle(self.objects)

        self.seen = [False] * len(self.objects)
        self.boat_pos = np.array(convert_image_coords(40, 220), dtype=np.float32)
        self.boat_heading = np.pi / 2
        self.other_obstacles = [
            [(53.0, 40.0), (62.0, 23.0), (60.0, 21.0), (51.0, 38.0)],
            [(50.0, 9.0), (50.0, 7.0), (48.0, 7.0), (48.0, 9.0)],
            [(33.0, 13.0), (33.0, 15.0), (30.0, 15.0), (30.0, 13.0)]
        ]

        self.waypoint_sub = rospy.Subscriber('arcturus_pilot/processed_waypoint', ProcessedWaypoint, self.waypoint_callback)
        self.pose_pub = rospy.Publisher('arcturus_pilot/pose', PoseStamped, queue_size=10)
        self.objects_pub = rospy.Publisher('sensor_suite/objects', ObjectArray, queue_size=5)
        self.occupancy_pub = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=5)
        self.waypoint_reached_pub = rospy.Publisher('arcturus_pilot/waypoint_reached', WaypointReached, queue_size=10)
        rospy.Subscriber('arcturus_pilot/waypoint_skip', SkipWaypoint, self.waypoint_skip_callback)

        self.waypoints = []
        self.skipped_waypoints = []
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
                running = True
                key_pressed = False
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        key_pressed = True
                        break
                    elif event.type == pygame.QUIT:
                        running = False
                        break
                if key_pressed or not running or rospy.is_shutdown():
                    break
            if not running:
                break

    def draw(self):
        self.screen.fill((255, 255, 255))
        pygame.draw.circle(self.screen, (0, 0, 255), get_screen_coords(self.boat_pos[0], self.boat_pos[1]), 5)

        waypoint, status = self.get_curr_waypoint()
        # rospy.logerr(str(waypoint))
        if waypoint != None:
            pygame.draw.circle(self.screen, (255, 0, 255) if status else (0, 0, 0), get_screen_coords(waypoint[0], waypoint[1]), 5)
        for object in self.objects:
            x, y = object[1], object[2]
            color = (0, 0, 0)
            if object[0][0] == 0:
                color = (255, 0, 0)
            elif object[0][0] == 1:
                color = (0, 255, 0)
            elif object[0][0] == 2:
                color = (255, 255, 0)
            elif object[0][0] == 3:
                color = (0, 0, 255)
            pygame.draw.circle(self.screen, color, get_screen_coords(x, y), 5)

        for obstacle in self.other_obstacles:
            converted_obstacle = [get_screen_coords(x, y) for (x, y) in obstacle]
            pygame.draw.polygon(self.screen, (255, 0, 0), converted_obstacle)

        pygame.display.flip()

    def waypoint_callback(self, waypoint):
        if waypoint.is_temp:
            self.temp_waypoint = (waypoint.x, waypoint.y, waypoint.heading)
            return
            
        while len(self.waypoints) <= waypoint.order:
            self.waypoints.append(())

        self.waypoints[waypoint.order] = (waypoint.x, waypoint.y, waypoint.heading)

    def waypoint_skip_callback(self, waypoint_skip):
        order = waypoint_skip.order
        self.skipped_waypoints.append(order)
        
    def get_curr_waypoint(self):
        while self.current_order in self.skipped_waypoints:
            # make sure the local planner is up to date with which waypoint is next
            self.waypoint_reached_pub.publish(WaypointReached(self.current_order))
            self.current_order += 1

        if self.temp_waypoint is not None:
            return self.temp_waypoint, True
        if len(self.waypoints) <= self.current_order:
            return None, False
        return self.waypoints[self.current_order], False

    def step(self):
        waypoint, _ = self.get_curr_waypoint()
        if waypoint is None:
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
        for ix, object in enumerate(self.objects):
            if np.linalg.norm(np.array([object[1], object[2]]) - self.boat_pos) <= CAMERA_DIST:
                self.seen[ix] = True

            if self.seen[ix]:
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
        occupancy_ros.header.frame_id = 'map'
        occupancy_ros.info.resolution = OCCUPANCY_RESOLUTION
        occupancy_ros.info.width = OCCUPANCY_WIDTH
        occupancy_ros.info.height = OCCUPANCY_HEIGHT

        grid = np.zeros((OCCUPANCY_HEIGHT, OCCUPANCY_WIDTH), dtype=np.int8)

        for object in self.objects:
            object_data = skimage.draw.ellipse(convert(object[2]), convert(object[1]), 
                convert(BUOY_RADIUS), convert(BUOY_RADIUS), shape=(OCCUPANCY_HEIGHT, OCCUPANCY_WIDTH))
            grid[object_data] = 100

        for obstacle in self.other_obstacles:
            obstacle_data = skimage.draw.polygon([convert(y) for (x, y) in obstacle], [convert(x) for (x, y) in obstacle], 
                (OCCUPANCY_HEIGHT, OCCUPANCY_WIDTH))
            grid[obstacle_data] = 100

        occupancy_ros.data = np.ndarray.tolist(np.ndarray.flatten(grid))
        self.occupancy_pub.publish(occupancy_ros)

    def send_info(self):
        self.send_objects()
        self.send_occupancy()
        
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = '/map'
        pose.pose.position = Point(x = self.boat_pos[0], y = self.boat_pos[1], z = 0)
        pose.pose.orientation = quaternion_from_angle(self.boat_heading)

        self.pose_pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('waypoint_sim')
    # rospy.logerr(MAP_WIDTH)
    # rospy.logerr(MAP_HEIGHT)
    # rospy.logerr(SCREEN_WIDTH)
    # rospy.logerr(SCREEN_HEIGHT)
    waypoint_sim = WaypointSim()
    waypoint_sim.run()
