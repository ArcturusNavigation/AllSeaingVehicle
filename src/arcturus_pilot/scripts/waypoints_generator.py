#!/usr/bin/env python

import rospy
import numpy as np
from enum import Enum
from arcturus_pilot.msg import RawWaypoint, WaypointReached, SkipWaypoint
from sensor_suite.msg import ObjectArray
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Point

from object_types import ObjectType, getObjectType
from geom_helper import angle_from_dir, sort_buoys_by_dir
from tf.transformations import euler_from_quaternion

# TODO: investigate what happens when you try to send a waypoint to the boat that is behind it with the same heading
# as the current boat. If the boat just tries to go backwards, then have to adjust the local planner code so that the
# intermediate waypoints have the correct heading. If not, then have to redo the code for generating waypoints to get
# out of dock

BUOY_DIST_THRESHOLD = 1 # distance buoys have to be from each other to be considered the same buoy
SNACK_RUN_MARKER_CLEARANCE_DIST = 1 # distance the boat will attempt to go past the snack run marker
SNACK_RUN_BEYOND_GOAL_DIST = 3 # distance the boat will go past the snack run opening / closing


SEARCH_EXPLORE_DIST = 5 # for the linear search behaviors, how far in a direction the search will go
SEARCH_TURN_AROUND_FORWARD_RADIUS = 2 # radius of the circle used for turning around forward
SEARCH_TURN_AROUND_BACKWARD_DIST = 2 # distance used for the turn around backward behavior
SEARCH_CIRCLE_RADIUS = 2 # radius of the circle in the search circle behavior
SEARCH_TIMEOUT = 15 # seconds the boat will spend searching before giving up and skipping the task

DOCK_TASKS_CLEARING_DIST = 3.0 # how far away from the dock task the boat will go before it stops to line up with the dock
# direction vectors for the dock tasks that give the direction the boat should face to complete them 
WATER_BLAST_DIST = 1.0 # how far away the boat should stop from the center of the water blast
SKEEBALL_DIST = 1.0 # how far away the boat should stop from the center of the skeeball

FIND_SEAT_DIR = np.array([1.0, 1.0])
SKEEBALL_DIR = np.array([0.0, 1.0])
WATER_BLAST_DIR = np.array([-1.0, 0.0])

FIND_SEAT_DIR /= np.linalg.norm(FIND_SEAT_DIR)
SKEEBALL_DIR /= np.linalg.norm(SKEEBALL_DIR)
WATER_BLAST_DIR /= np.linalg.norm(WATER_BLAST_DIR)

class State(Enum):
    CHANNEL = 0,
    AVOID_CROWD = 1,
    SNACK_RUN = 2,
    FIND_SEAT = 3,
    WATER_BLAST = 4,
    SKEEBALL = 5,
    RETURN = 6,
    FINISHED = 7,
    SEARCH = 8

class SearchBehavior(Enum):
    STEER_RIGHT = 0,
    STEER_LEFT = 1,
    FORWARD = 2, # TESTED
    CIRCLE = 3,
    TURN_AROUND_FORWARD = 4, # BROKEN (?)
    TURN_AROUND_BACKWARD = 5, # TESTED
    SKIP = 6

class SearchData():
    def __init__(self, next_state, search_behavior, init_pos, init_heading):
        self.next_state = next_state
        self.search_behavior = search_behavior
        self.init_pos = np.copy(init_pos)
        self.init_heading = init_heading

        if next_state == State.CHANNEL:
            self.search_targets = [ObjectType.CHANNEL_GREEN, ObjectType.CHANNEL_RED]

        elif next_state == State.AVOID_CROWD:
            self.search_targets = [ObjectType.AVOID_GREEN, ObjectType.AVOID_RED]
            
        elif next_state == State.SNACK_RUN:
            self.search_targets = [ObjectType.SNACK_GREEN, ObjectType.SNACK_RED]

        elif next_state == State.RETURN:
            self.search_targets = [ObjectType.RETURN, ObjectType.RETURN]

        elif next_state == State.FIND_SEAT:
            self.search_targets = [ObjectType.FIND_SEAT]
            
        elif next_state == State.WATER_BLAST:
            self.search_targets = [ObjectType.WATER_BLAST]
            
        elif next_state == State.SKEEBALL:
            self.search_targets = [ObjectType.SKEEBALL]

    def has_found_targets(self, objects):
        """
        Returns whether or not all of the search targets have been found
        """
        used = []
        for search_target in self.search_targets:
            found = False
            for ix, object in enumerate(objects):
                if object[0] == search_target and ix not in used:
                    found = True
                    used.append(ix)
                    break
            if not found:
                return False
        return True

    def get_waypoints(self):
        waypoints = []
        waypoints_id_prefix = 'search_' + str(self.next_state.value) + '_'
        if self.search_behavior == SearchBehavior.STEER_RIGHT:
            target_heading = self.init_heading + np.pi / 4
            target_pos = self.init_pos + np.array([np.cos(target_heading), np.sin(target_heading)]) * SEARCH_EXPLORE_DIST
            waypoints.append((target_pos, target_heading, waypoints_id_prefix + '0', False))

        elif self.search_behavior == SearchBehavior.STEER_LEFT:
            target_heading = self.init_heading - np.pi / 4
            target_pos = self.init_pos + np.array([np.cos(target_heading), np.sin(target_heading)]) * SEARCH_EXPLORE_DIST
            waypoints.append((target_pos, target_heading, waypoints_id_prefix + '0', False))

        elif self.search_behavior == SearchBehavior.FORWARD:
            target_pos = self.init_pos + np.array([np.cos(self.init_heading), np.sin(self.init_heading)]) * SEARCH_EXPLORE_DIST
            waypoints.append((target_pos, self.init_heading, waypoints_id_prefix + '0', False))

        elif self.search_behavior == SearchBehavior.CIRCLE:
            for i in range(8):
                dir = self.init_heading + i * np.pi / 4
                heading = dir + np.pi / 2
                target_pos = self.init_pos + np.array([np.cos(dir), np.sin(dir)]) * SEARCH_CIRCLE_RADIUS
                waypoints.append((target_pos, heading, waypoints_id_prefix + str(i), False))

        elif self.search_behavior == SearchBehavior.TURN_AROUND_FORWARD:
            target_heading_1 = self.init_heading + np.pi / 2
            dir_1 = self.init_heading + np.pi / 4
            target_pos_1 = self.init_pos + np.array([np.cos(dir_1), np.sin(dir_1)]) * SEARCH_TURN_AROUND_FORWARD_RADIUS
            waypoints.append((target_pos_1, target_heading_1, waypoints_id_prefix + '0', False))

            target_heading_2 = self.init_heading + np.pi
            dir_2 = self.init_heading + np.pi / 2
            target_pos_2 = self.init_pos + np.array([np.cos(dir_2), np.sin(dir_2)]) * SEARCH_TURN_AROUND_FORWARD_RADIUS
            waypoints.append((target_pos_2, target_heading_2, waypoints_id_prefix + '1', False))

        elif self.search_behavior == SearchBehavior.TURN_AROUND_BACKWARD:
            target_heading_1 = self.init_heading + np.pi / 4
            dir_1 = self.init_heading + np.pi / 4
            target_pos_1 = self.init_pos - np.array([np.cos(dir_1), np.sin(dir_1)]) * SEARCH_TURN_AROUND_BACKWARD_DIST
            waypoints.append((target_pos_1, target_heading_1, waypoints_id_prefix + '0', False))

            target_heading_2 = self.init_heading + np.pi
            dir_2 = self.init_heading + np.pi / 2
            target_pos_2 = self.init_pos - np.array([np.cos(dir_2), np.sin(dir_2)]) * SEARCH_TURN_AROUND_BACKWARD_DIST
            waypoints.append((target_pos_2, target_heading_2, waypoints_id_prefix + '1', False))

        return waypoints

"""
Generates raw waypoints based on the current task
Sends these raw waypoints to the arcturus_pilot/local_planner node, which processes them and then sends them to
arcturus_pilot/waypoint_pilot

Receives data from the sensor suite with the locations of all of the buoys (expected in the local_origin frame)
All waypoints are sent in the local_origin frame
"""

class WaypointGenerator():
    def __init__(self):
        self.curr_pos = np.array([0, 0])
        self.curr_heading = 0
        self.start_pos = None
        self.last_waypoint_visited = -1

        self.curr_objects = []
        self.waypoints_sent = []
        self.curr_state = None    
        self.search_data = None

        self.find_seat_target = None
        self.water_gun_status = None
        self.skeeball_status = None

        self.waypoint_pub = rospy.Publisher('arcturus_pilot/raw_waypoint', RawWaypoint, queue_size=10)
        self.reached_waypoint_sub = rospy.Subscriber('arcturus_pilot/waypoint_reached', WaypointReached, self.reached_waypoint_callback)
        self.waypoint_skip_pub = rospy.Publisher('arcturus_pilot/waypoint_skip', SkipWaypoint)
        self.pose_sub = rospy.Subscriber('arcturus_pilot/pose', PoseStamped, self.pose_callback)
        self.objects_sub = rospy.Subscriber('sensor_suite/objects', ObjectArray, self.objects_callback)

        # TODO replace these placeholders
        self.find_seat_ready_pub = rospy.Publisher('find_seat/ready', Empty, queue_size = 10)
        self.water_gun_ready_pub = rospy.Publisher('water_gun/ready', Empty, queue_size = 10)
        self.skeeball_ready_pub = rospy.Publisher('skeeball/ready', Empty, queue_size = 10)

        self.find_seat_target_sub = rospy.Subscriber('find_seat/target', Point, self.find_seat_target_callback)
        self.water_gun_status_sub = rospy.Subscriber('water_gun/status', Empty, self.water_gun_status_callback)
        self.skeeball_status_sub = rospy.Subscriber('skeeball/status', Empty, self.skeeball_status_callback)

    #############################################################
    ################## STATE MACHINE MANAGEMENT #################
    #############################################################

    def go_next_state(self):
        """
        Sets up the next state given our current state
        Defines our path through the course
        """

        if self.curr_state == State.CHANNEL:
            self.search(State.AVOID_CROWD, SearchBehavior.FORWARD)

        elif self.curr_state == State.AVOID_CROWD:
            self.search(State.SNACK_RUN, SearchBehavior.STEER_RIGHT)
    
        elif self.curr_state == State.SNACK_RUN:
            self.search(State.FIND_SEAT, SearchBehavior.FORWARD)

        elif self.curr_state == State.FIND_SEAT:
            self.search(State.WATER_BLAST, SearchBehavior.TURN_AROUND_BACKWARD)

        elif self.curr_state == State.WATER_BLAST:
            self.search(State.SKEEBALL, SearchBehavior.STEER_RIGHT)

        elif self.curr_state == State.SKEEBALL:
            self.search(State.RETURN)

        elif self.curr_state == State.RETURN:
            self.curr_state = State.FINISHED

    def search(self, next_state, search_behavior=SearchBehavior.SKIP):
        """
        Given the next desired state, sets up searching for the next state 
        """

        self.curr_state = State.SEARCH
        self.search_data = SearchData(next_state, search_behavior, self.curr_pos, self.curr_heading)
        rospy.Timer(rospy.Duration.from_sec(SEARCH_TIMEOUT), self.search_timeout_callback, True)

    def search_timeout_callback(self, data):
        self.curr_state = self.search_data.next_state

    #############################################################
    ################## ROS SUBSCRIBER CALLBACKS #################
    #############################################################

    def objects_callback(self, data):
        self.curr_objects = np.zeros((0, 3))

        for object in data.objects:
            self.curr_objects = np.append(self.curr_objects, [[getObjectType(object.label, object.task_label), 
                object.pos.x, object.pos.y]], axis=0)

    def pose_callback(self, data):
        self.curr_pos[0] = data.pose.position.x
        self.curr_pos[1] = data.pose.position.y
        explicit_quat = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        self.curr_heading = euler_from_quaternion(explicit_quat)[2]
        # rospy.logerr(str(self.curr_heading))

        if self.start_pos is None:
            self.start_pos = np.copy(self.curr_pos)

    def reached_waypoint_callback(self, data):
        self.last_waypoint_visited = data.order

    def find_seat_target_callback(self, data):
        self.find_seat_target = data

    def skeeball_status_callback(self, data):
        self.skeeball_status = True
    
    def water_gun_status_callback(self, data):
        self.water_gun_status = True
        
    #############################################################
    ################# WAYPOINT SENDING FUNCTIONS ################
    #############################################################

    def find_index(self, waypoint_id):
        for ix, waypoint in enumerate(self.waypoints_sent):
            if waypoint[1] == waypoint_id:
                return ix
        return -1

    def send_waypoint(self, position, direction, waypoint_id, is_dir_vector=True):
        """
        Send a waypoint to the arcturus_pilot node with the given position and direction.
        Checks if the waypoint is already visited to ensure the correct index is sent
        """

        # check if this was already sent as a waypoint
        ix = self.find_index(waypoint_id)
        if ix == -1:
            self.waypoints_sent.append((position, waypoint_id))
            ix = len(self.waypoints_sent) - 1
        else:
            self.waypoints_sent[ix] = (position, waypoint_id)

        self.waypoint_pub.publish(RawWaypoint(position[0], position[1], angle_from_dir(direction) if is_dir_vector else direction, ix))
    
    def skip_waypoint(self, waypoint_id):
        ix = self.find_index(waypoint_id)
        if ix != -1:
            self.waypoint_skip_pub.publish(SkipWaypoint(ix))

    def send_midpoint(self, buoy_1, buoy_2, waypoint_id):
        """
        Send the midpoint of two buoys as a waypoint with a heading pointing away from the current position
        """
        midpoint = (buoy_1 + buoy_2) / 2.0
        dir = (buoy_1 - buoy_2) / np.linalg.norm(buoy_1 - buoy_2)
        dir = np.array([dir[1], -dir[0]])
        test_pos = midpoint + dir * 10
        test_neg = midpoint - dir * 10
        heading_dir = dir if np.linalg.norm(self.curr_pos - test_pos) > np.linalg.norm(self.curr_pos - test_neg) else -dir

        self.send_waypoint(midpoint, heading_dir, waypoint_id)

    def is_last_visited(self, waypoint_id):
        return self.last_waypoint_visited != -1 and self.waypoints_sent[self.last_waypoint_visited][1] == waypoint_id

        
    #############################################################
    ################## STATE MACHINE MAIN LOOP ##################
    #############################################################
    def run(self):
        # wait until we get some data about current position
        rate = rospy.Rate(10)
        while self.start_pos is None and not rospy.is_shutdown():
            rate.sleep() 

        self.search(State.CHANNEL, SearchBehavior.FORWARD)
    
        while not rospy.is_shutdown():
            objects = np.array(self.curr_objects)

            # rospy.logerr(str(self.curr_heading))
            # rospy.logerr(str(self.start_pos))
            # rospy.logerr(str(self.last_waypoint_visited))

            if self.curr_state == State.CHANNEL:
                red_buoys = []
                green_buoys = []

                for object in objects:
                    if object[0] == ObjectType.CHANNEL_RED:
                        red_buoys.append(object[1:])
                    elif object[0] == ObjectType.CHANNEL_GREEN:
                        green_buoys.append(object[1:])

                red_buoys = sorted(red_buoys, key=lambda buoy: np.linalg.norm(buoy - self.start_pos))
                green_buoys = sorted(green_buoys, key=lambda buoy: np.linalg.norm(buoy - self.start_pos))

                if len(red_buoys) >= 1 and len(green_buoys) >= 1:
                    self.send_midpoint(red_buoys[0], green_buoys[0], 'channel0')
                if len(red_buoys) >= 2 and len(green_buoys) >= 2:
                    self.send_midpoint(red_buoys[1], green_buoys[1], 'channel1')

                if self.is_last_visited('channel1'):
                    self.go_next_state()
                
            elif self.curr_state == State.AVOID_CROWD:
                red_buoys = []
                green_buoys = []

                for object in objects:
                    if object[0] == ObjectType.AVOID_RED:
                        red_buoys.append(object[1:])
                    elif object[0] == ObjectType.AVOID_GREEN:
                        green_buoys.append(object[1:])

                # if we haven't found anything, then just skip (should never happen tbh)
                if len(red_buoys) == 0 or len(green_buoys) == 0:
                    self.go_next_state()
                else:
                    sorted_red = sort_buoys_by_dir(np.array(red_buoys))
                    sorted_green = sort_buoys_by_dir(np.array(green_buoys))

                    num_waypoints = min(len(sorted_red), len(sorted_green))
                    for i in range(num_waypoints):
                        self.send_midpoint(sorted_red[i], sorted_green[i], 'avoid_crowd' + str(i))

                    if self.is_last_visited('avoid_crowd' + str(num_waypoints - 1)):
                        self.go_next_state()

            elif self.curr_state == State.SNACK_RUN:
                red_buoy = None
                green_buoy = None
                mark_buoy = None

                for object in objects:
                    if object[0] == ObjectType.SNACK_RED:
                        red_buoy = object[1:]
                    elif object[0] == ObjectType.SNACK_GREEN:
                        green_buoy = object[1:]
                    elif object[0] == ObjectType.SNACK_BLUE:
                        mark_buoy = object[1:]

                # if we haven't found anything, then just skip
                if red_buoy is None and green_buoy is None:
                    self.go_next_state()
                else:
                    # estimate the red position using green buoy
                    if red_buoy is None:
                        red_buoy = green_buoy - np.array(3.0)

                    # estimate the red position using green buoy
                    if green_buoy is None:
                        green_buoy = red_buoy + np.array(3.0)

                    midpoint = (red_buoy + green_buoy) / 2.0
                    dir = red_buoy - green_buoy
                    dir /= np.linalg.norm(dir)
                    perp = np.array([-dir[1], dir[0]])

                    # estimate the position of the mark buoy (might have to change depending on the course)
                    if mark_buoy is None:
                        second_perp = np.copy(perp)
                        if np.linalg.norm(self.curr_pos - (midpoint + perp)) > np.linalg.norm(self.curr_pos - (midpoint - perp)):
                            second_perp *= -1
                        mark_buoy = midpoint - perp * 15.0 # estimate as 15 meters along the perpendicular pointing away from the current position

                    positions = []
                    directions = []

                    if np.linalg.norm(mark_buoy - (midpoint + perp)) > np.linalg.norm(mark_buoy - (midpoint - perp)):
                        perp *= -1

                    positions.append(midpoint - perp * SNACK_RUN_BEYOND_GOAL_DIST)
                    positions.append(midpoint)
                    positions.append(midpoint + perp * SNACK_RUN_BEYOND_GOAL_DIST)

                    positions.append(mark_buoy + dir * SNACK_RUN_MARKER_CLEARANCE_DIST)
                    positions.append(mark_buoy + perp * SNACK_RUN_MARKER_CLEARANCE_DIST)
                    positions.append(mark_buoy - dir * SNACK_RUN_MARKER_CLEARANCE_DIST)

                    positions.append(midpoint)
                    positions.append(midpoint - perp * SNACK_RUN_BEYOND_GOAL_DIST)

                    for self.curr_pos, next_pos in zip(positions[:-1], positions[1:]):
                        dir = next_pos - self.curr_pos
                        dir /= np.linalg.norm(dir)
                        directions.append(dir)
                    directions.append(-perp)

                    last_ix = -1
                    for ix, (point, direction) in enumerate(zip(positions, directions)):
                        self.send_waypoint(point, direction, 'snack_run' + str(ix))
                        last_ix = ix

                    if self.is_last_visited('snack_run' + str(last_ix)):
                        self.go_next_state()

            elif self.curr_state == State.FIND_SEAT:
                find_seat_pos = None

                for object in objects:
                    if object[0] == ObjectType.FIND_SEAT:
                        find_seat_pos = object[1:]
                
                if find_seat_pos is None:
                    self.go_next_state()
                else:
                    self.send_waypoint(find_seat_pos - FIND_SEAT_DIR * DOCK_TASKS_CLEARING_DIST, FIND_SEAT_DIR, 'find_seat0')

                    if self.is_last_visited('find_seat0'):
                        self.find_seat_ready_pub.publish(Empty())

                    if self.find_seat_target != None:
                        self.send_waypoint((self.find_seat_target.x, self.find_seat_target.y), FIND_SEAT_DIR, 'find_seat1')
                        self.send_waypoint(find_seat_pos - FIND_SEAT_DIR * DOCK_TASKS_CLEARING_DIST, FIND_SEAT_DIR, 'find_seat2')
                        
                    if self.is_last_visited('find_seat2'):
                        self.go_next_state()

            elif self.curr_state == State.WATER_BLAST:
                water_blast_pos = None

                for object in objects:
                    if object[0] == ObjectType.WATER_BLAST:
                        water_blast_pos = object[1:]
                
                if water_blast_pos is None:
                    self.go_next_state()
                else:
                    self.send_waypoint(water_blast_pos - WATER_BLAST_DIR * DOCK_TASKS_CLEARING_DIST, WATER_BLAST_DIR, 'water_blast0')
                    self.send_waypoint(water_blast_pos - WATER_BLAST_DIR * WATER_BLAST_DIST, WATER_BLAST_DIR, 'water_blast1')

                    if self.is_last_visited('water_blast1'):
                        self.water_gun_ready_pub.publish(Empty())
                    
                    if self.water_gun_status != None:
                        self.send_waypoint(water_blast_pos - WATER_BLAST_DIR * DOCK_TASKS_CLEARING_DIST, WATER_BLAST_DIR, 'water_blast2')
                        
                    if self.is_last_visited('water_blast2'):
                        self.go_next_state()

            elif self.curr_state == State.SKEEBALL:
                skeeball_pos = None

                for object in objects:
                    if object[0] == ObjectType.SKEEBALL:
                        skeeball_pos = object[1:]
                
                if skeeball_pos is None:
                    self.go_next_state()
                else:
                    self.send_waypoint(skeeball_pos - SKEEBALL_DIR * DOCK_TASKS_CLEARING_DIST, SKEEBALL_DIR, 'skeeball0')
                    self.send_waypoint(skeeball_pos - SKEEBALL_DIR * SKEEBALL_DIST, SKEEBALL_DIR, 'skeeball1')

                    if self.is_last_visited('skeeball1'):
                        self.skeeball_ready_pub.publish(Empty())
                    
                    if self.skeeball_status != None:
                        self.send_waypoint(skeeball_pos - SKEEBALL_DIR * DOCK_TASKS_CLEARING_DIST, SKEEBALL_DIR, 'skeeball2')
                        
                    if self.is_last_visited('skeeball2'):
                        self.go_next_state()

            elif self.curr_state == State.RETURN:
                return_buoys = []
                for object in objects:
                    if object[0] == ObjectType.RETURN:
                        return_buoys.append(object[1:])

                if len(return_buoys) < 2:
                    self.go_next_state()
                else:
                    self.send_midpoint(return_buoys[0], return_buoys[1], 'return')

                    if self.is_last_visited('return'):
                        self.go_next_state()
                        
            elif self.curr_state == State.SEARCH:
                if self.search_data.has_found_targets(objects):
                    for waypoint_data in self.search_data.get_waypoints():
                        self.skip_waypoint(waypoint_data[2])
                    self.curr_state = self.search_data.next_state
                else:
                    for waypoint_data in self.search_data.get_waypoints():
                        self.send_waypoint(waypoint_data[0], waypoint_data[1], waypoint_data[2], waypoint_data[3])

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('waypoints_generator')
    waypoint_generator = WaypointGenerator()
    waypoint_generator.run()
