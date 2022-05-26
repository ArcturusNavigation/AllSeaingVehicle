#!/usr/bin/env python

import rospy
import numpy as np
from enum import Enum
from arcturus_pilot.msg import Waypoint, WaypointReached
from buoy_types import BuoyType
from geometry_msgs.msg import PoseStamped
from geom_helper import angle_from_dir, sort_buoys_by_dir
from tf import euler_from_quaternion

from arcturus_pilot.msg import Buoy, BuoyList

BUOY_DIST_THRESHOLD = 1 # distance buoys have to be from each other to be considered the same buoy
SNACK_RUN_MARKER_CLEARANCE_DIST = 1 # distance the boat will attempt to go past the snack run marker
SNACK_RUN_BEYOND_GOAL_DIST = 3 # distance the boat will go past the snack run opening / closing

SEARCH_EXPLORE_DIST = 5
SEARCH_TURN_AROUND_DIST = 1
SEARCH_CIRCLE_DIST = 2

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
    FORWARD = 2,
    CIRCLE = 3,
    TURN_AROUND = 4

class SearchData():
    def __init__(self, next_state, search_behavior, init_pos, init_heading):
        self.next_state = next_state
        self.search_behavior = search_behavior
        self.init_pos = init_pos
        self.init_heading = init_heading

        if next_state == State.CHANNEL:
            self.search_targets = [BuoyType.CHANNEL_GREEN, BuoyType.CHANNEL_RED]
        elif next_state == State.AVOID_CROWD:
            self.search_targets = [BuoyType.AVOID_GREEN, BuoyType.AVOID_RED]
        elif next_state == State.SNACK_RUN:
            self.search_targets = [BuoyType.SNACK_GREEN, BuoyType.SNACK_RED]
        elif next_state == State.RETURN:
            self.search_targets = [BuoyType.RETURN, BuoyType.RETURN]

    def has_found_targets(self, buoys):
        """
        Returns whether or not all of the search targets have been found
        """
        used = []
        for search_buoy in self.search_targets:
            found = False
            for buoy in buoys:
                if buoy[0] == search_buoy and buoy not in used:
                    found = True
                    used.append(buoy)
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
                target_pos = self.init_pos + np.array([np.cos(dir), np.sin(dir)]) * SEARCH_CIRCLE_DIST
                waypoints.append((target_pos, heading, waypoints_id_prefix + str(i), False))

        elif self.search_behavior == SearchBehavior.TURN_AROUND:
            target_heading_1 = self.init_heading + np.pi / 2
            dir_1 = self.init_heading + np.pi / 4
            target_pos_1 = self.init_pos + np.array([np.cos(dir_1), np.sin(dir_1)]) * SEARCH_TURN_AROUND_DIST
            waypoints.append((target_pos_1, target_heading_1, waypoints_id_prefix + '0', False))

            target_heading_2 = self.init_heading + np.pi
            dir_2 = self.init_heading + np.pi / 2
            target_pos_2 = self.init_pos + np.array([np.cos(dir_2), np.sin(dir_2)]) * SEARCH_TURN_AROUND_DIST
            waypoints.append((target_pos_2, target_heading_2, waypoints_id_prefix + '1', False))

        return waypoints

"""
Sends waypoints based on the current task to arcturus_pilot/waypoint_pilot node
Receives data from the sensor suite with the locations of all of the buoys (expected in the local_origin frame)
All waypoints are sent in the local_origin frame
"""
def main():
    rospy.init_node('waypoints_generator')
    
    curr_pos = np.array([0, 0])
    curr_heading = 0
    last_waypoint_visited = -1

    curr_buoys = []
    waypoints_sent = []
    curr_state = State.CHANNEL
    
    search_data = None

    def search(next_state):
        """
        Given the next desired state, sets up searching for the next state 
        """
        nonlocal curr_state
        nonlocal search_data

        curr_state = State.SEARCH
        search_behavior = SearchBehavior.FORWARD

        if next_state == State.CHANNEL:
            search_behavior = SearchBehavior.FORWARD
        elif next_state == State.AVOID_CROWD:
            search_behavior = SearchBehavior.STEER_LEFT
        elif next_state == State.SNACK_RUN:
            search_behavior = SearchBehavior.STEER_RIGHT
        elif next_state == State.RETURN:
            search_behavior = SearchBehavior.FORWARD
        
        search_data = SearchData(next_state, search_behavior, curr_pos, curr_heading)

    def go_next_state():
        """
        Sets up the next state given our current state
        Essentially defines our path through the course
        """
        nonlocal curr_state

        if curr_state == State.CHANNEL:
            search(State.AVOID_CROWD)

        elif curr_state == State.AVOID_CROWD:
            search(State.SNACK_RUN)
    
        elif curr_state == State.SNACK_RUN:
            return State.FIND_SEAT

        elif curr_state == State.FIND_SEAT:
            return State.WATER_BLAST

        elif curr_state == State.WATER_BLAST:
            return State.SKEEBALL

        elif curr_state == State.SKEEBALL:
            search(State.RETURN)

        elif curr_state == State.RETURN:
            return State.FINISHED

    def buoys_callback(data):
        nonlocal curr_buoys
        visible_buoys = data.buoys
        prev_buoys = curr_buoys
        curr_buoys = np.zeros((0, 3))

        for buoy in visible_buoys:
            curr_buoys = np.append(curr_buoys, [[buoy.type, buoy.x, buoy.y]], axis=0)

        for prev_buoy in prev_buoys:
            matched = False
            for cur_buoy in visible_buoys:
                if prev_buoy[0] == cur_buoy[0] and np.linalg.norm(prev_buoy - cur_buoy) < BUOY_DIST_THRESHOLD:
                    matched = True
                    break
            if not matched:
                np.append(curr_buoys, prev_buoy[np.newaxis, :], axis=0)

    def pos_callback(data):
        nonlocal curr_pos
        nonlocal curr_heading
        curr_pos[0] = data.pose.position.x
        curr_pos[1] = data.pose.position.y
        curr_heading = euler_from_quaternion(data.pose.orientation)[2]

    def reached_waypoint_callback(data):
        nonlocal last_waypoint_visited
        nonlocal curr_state
        last_waypoint_visited = data.order

    waypoint_pub = rospy.publisher('arcturus_pilot/waypoint', Waypoint, queue_size=10)
    reached_waypoint_sub = rospy.Subscriber('arcturus_pilot/waypoint_reached', WaypointReached, reached_waypoint_callback)
    pos_sub = rospy.Subscriber('arcturus_pilot/position', PoseStamped, pos_callback)
    buoys_sub = rospy.Subscriber('sensor_suite/buoys', BuoyList, buoys_callback)

    def find_index(waypoint_id):
        for ix, waypoint in enumerate(waypoints_sent):
            if waypoint[1] == waypoint_id:
                return ix
        return -1

    def send_waypoint(position, direction, waypoint_id, direction_is_angle=True):
        """
        Send a waypoint to the arcturus_pilot node with the given position and direction.
        Checks if the waypoint is already visited to ensure the correct index is sent
        """

        # check if this was already sent as a waypoint
        ix = find_index(waypoint_id)
        if ix == -1:
            waypoints_sent.append((position, waypoint_id))
            ix = len(waypoints_sent) - 1
        else:
            waypoints_sent[ix] = (position, waypoint_id)

        waypoint_pub.publish(Waypoint(position[0], position[1], angle_from_dir(direction) if direction_is_angle else direction, ix))
    
    def send_midpoint(buoy_1, buoy_2, waypoint_id):
        """
        Send the midpoint of two buoys as a waypoint with a heading pointing away from the current position
        """
        midpoint = (buoy_1 + buoy_2) / 2.0
        dir = (buoy_1 - buoy_2) / np.linalg.norm(buoy_1 - buoy_2)
        dir = np.array([dir[1], -dir[0]])
        test_pos = midpoint + dir * 10
        test_neg = midpoint - dir * 10
        heading_dir = dir if np.linalg.norm(curr_pos - test_pos) > np.linalg.norm(curr_pos - test_neg) else -dir

        send_waypoint(midpoint, heading_dir, waypoint_id)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        buoys = np.array(curr_buoys)

        if curr_state == State.CHANNEL:
            red_buoys = []
            green_buoys = []

            for buoy in buoys:
                if buoy[0] == BuoyType.CHANNEL_RED:
                    red_buoys.append(buoy[1:])
                elif buoy[0] == BuoyType.CHANNEL_GREEN:
                    green_buoys.append(buoy[1:])

            red_buoys = sorted(red_buoys, key=lambda buoy: np.linalg.norm(buoy - curr_pos))
            green_buoys = sorted(green_buoys, key=lambda buoy: np.linalg.norm(buoy - curr_pos))

            if len(red_buoys) >= 1 and len(green_buoys) >= 1:
                send_midpoint(red_buoys[0], green_buoys[0], 'channel0')
            if len(red_buoys) >= 2 and len(green_buoys) >= 2:
                send_midpoint(red_buoys[1], green_buoys[1], 'channel1')

            if last_waypoint_visited != -1 and waypoints_sent[last_waypoint_visited][1] == 'channel1':
                go_next_state()
            
        elif curr_state == State.AVOID_CROWD:
            red_buoys = []
            green_buoys = []

            for buoy in buoys:
                if buoy[0] == BuoyType.AVOID_RED:
                    red_buoys.append(buoy[1:])
                elif buoy[0] == BuoyType.AVOID_GREEN:
                    green_buoys.append(buoy[1:])
            
            sorted_red = sort_buoys_by_dir(np.array(red_buoys))
            sorted_green = sort_buoys_by_dir(np.array(green_buoys))

            num_waypoints = min(len(sorted_red), len(sorted_green))
            for i in range(num_waypoints):
                send_midpoint(sorted_red[i], sorted_green[i], 'avoid_crowd' + str(i))

            if last_waypoint_visited != -1 and waypoints_sent[last_waypoint_visited][1] == 'avoid_crowd' + str(num_waypoints - 1):
                go_next_state()

        elif curr_state == State.SNACK_RUN:
            red_buoy = None
            green_buoy = None
            mark_buoy = None

            for buoy in buoys:
                if buoy[0] == BuoyType.SNACK_RED:
                    red_buoy = buoy[1:]
                elif buoy[0] == BuoyType.SNACK_GREEN:
                    green_buoy = buoy[1:]
                elif buoy[0] == BuoyType.SNACK_BLUE:
                    mark_buoy = buoy[1:]

            midpoint = (red_buoy + green_buoy) / 2.0
            dir = red_buoy - green_buoy
            dir /= np.linalg.norm(dir)
            perp = np.array([-dir[1], dir[0]])

            # estimate the position of the mark buoy (might have to change depending on the course)
            if mark_buoy == None:
                second_perp = np.copy(perp)
                if np.linalg.norm(curr_pos - (midpoint + perp)) > np.linalg.norm(curr_pos - (midpoint - perp)):
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

            for curr_pos, next_pos in zip(positions[:-1], positions[1:]):
                dir = next_pos - curr_pos
                dir /= np.linalg.norm(dir)
                directions.append(dir)
            directions.append(-perp)

            last_ix = -1
            for ix, (point, direction) in enumerate(zip(positions, directions)):
                send_waypoint(point, direction, 'snack_run' + str(ix))
                last_ix = ix

            if last_waypoint_visited != -1 and waypoints_sent[last_waypoint_visited][1] == 'snack_run' + str(last_ix):
                go_next_state()

        elif curr_state == State.FIND_SEAT:
            go_next_state()
        elif curr_state == State.WATER_BLAST:
            go_next_state()
        elif curr_state == State.SKEEBALL:
            go_next_state()
        elif curr_state == State.RETURN:
            return_buoys = []
            for buoy in buoys:
                if buoy[0] == BuoyType.RETURN:
                    return_buoys.append(buoy[1:])

            send_midpoint(return_buoys[0], return_buoys[1], 'return')

            if last_waypoint_visited != -1 and waypoints_sent[last_waypoint_visited][1] == 'return':
                go_next_state()
        elif curr_state == State.SEARCH:
            if search_data.has_found_targets(buoys):
                curr_state = search_data.next_state
            else:
                for waypoint_data in search_data.get_waypoints():
                    send_waypoint(*waypoint_data)

        rate.sleep()

if __name__ == '__main__':
    main()
