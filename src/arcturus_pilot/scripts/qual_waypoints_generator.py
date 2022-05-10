#!/usr/bin/env python

import rospy
import numpy as np
from enum import Enum
from arcturus_pilot.msg import Waypoint
from buoy_types import BuoyType
from geometry_msgs.msg import PoseStamped
from geom_helper import angle_from_dir
from tf import euler_from_quaternion

class State(Enum):
    CHANNEL = 0,
    AVOID_CROWD = 1,
    FIND_SNACK_RUN = 2,
    SNACK_RUN = 3,
    FIND_SEAT = 4,
    WATER_BLAST = 5,
    SKEEBALL = 6,
    RETURN = 7

WAYPOINT_DIST_THRESHOLD = 1
SNACK_RUN_MARKER_CLEARANCE_DIST = 1
SNACK_RUN_BEYOND_GOAL_DIST = 3

def main():
    rospy.init_node('waypoints_generator')
    
    cur_pos = np.array([0, 0])
    cur_heading = 0
    num_passed = 0
    num_left = 100

    visible_buoys = []
    waypoints_sent = []
    curr_state = State.CHANNEL
    prev_state = None

    # previous data so we can still compute waypoints when we can't see all of the buoys
    snack_run_prev_midpoint = None
    snack_run_prev_marker = None

    def pos_callback(data):
        nonlocal cur_pos
        nonlocal cur_heading
        cur_pos[0] = data.pose.position.x
        cur_pos[1] = data.pose.position.y
        cur_heading = euler_from_quaternion(data.pose.orientation)[2]

    def reached_waypoint_callback(data):
        nonlocal num_passed
        nonlocal num_left
        nonlocal curr_state
        num_passed += 1
        num_left -= 1

        if num_left == 0:
            if curr_state == State.CHANNEL:
                curr_state = State.AVOID_CROWD
            elif curr_state == State.AVOID_CROWD:
                curr_state = State.FIND_SNACK_RUN
            elif curr_state == State.SNACK_RUN:
                pass
            elif curr_state == State.FIND_SEAT:
                pass
            elif curr_state == State.WATER_BLAST:
                pass
            elif curr_state == State.SKEEBALL:
                pass
            elif curr_state == State.RETURN:
                pass

    waypoint_pub = rospy.publisher('arcturus_pilot/waypoint', Waypoint, queue_size=10)
    reached_waypoint_sub = rospy.Subscriber('arcturus_pilot/reached_waypoint', WaypointReached, reached_waypoint_callback)
    pos_sub = rospy.Subscriber('arcturus_pilot/position', PoseStamped, pos_callback)

    def find_closest_visited_index(point, wix):
        for ix, waypoint in enumerate(waypoints_sent):
            if np.linalg.norm(waypoint[0] - point) < WAYPOINT_DIST_THRESHOLD and waypoint[1] == wix:
                return ix
        return -1

    def send_waypoint(position, direction, wix=0):
        """
        Send a waypoint to the arcturus_pilot node with the given position and direction.
        Checks if the waypoint is already visited to ensure the correct index is sent
        """

        # check if this was already sent as a waypoint
        ix = find_closest_visited_index(position, wix)
        if ix == -1:
            waypoints_sent.append((position, wix))
            ix = len(waypoints_sent) - 1
        else:
            waypoints_sent[ix] = (position, wix)

        waypoint_pub.publish(Waypoint(position[0], position[1], angle_from_dir(direction), ix))
    
    def send_midpoint(buoy_1, buoy_2):
        # send the midpoint of two buoys as the waypoint with a heading pointing away from the current position
        midpoint = (buoy_1 + buoy_2) / 2.0
        dir = (buoy_1 - buoy_2) / np.linalg.norm(buoy_1 - buoy_2)
        dir = np.array([dir[1], -dir[0]])
        test_pos = midpoint + dir * 10
        test_neg = midpoint - dir * 10
        heading_dir = dir if np.linalg.norm(cur_pos - test_pos) > np.linalg.norm(cur_pos - test_neg) else -dir

        send_waypoint(midpoint, heading_dir)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        state_changed = curr_state != prev_state
        buoys = np.array(visible_buoys)

        if curr_state == State.CHANNEL:
            if state_changed:
                num_left = 2

            closest_red = None
            closest_green = None

            for buoy in buoys:
                if buoy[0] == BuoyType.CHANNEL_RED:
                    if closest_red is None:
                        closest_red = buoy
                    elif np.linalg.norm(buoy[1] - cur_pos) < np.linalg.norm(closest_red - cur_pos):
                        closest_red = buoy
                elif buoy[0] == BuoyType.CHANNEL_GREEN:
                    if closest_green is None:
                        closest_green = buoy
                    elif np.linalg.norm(buoy[1] - cur_pos) < np.linalg.norm(closest_green - cur_pos):
                        closest_green = buoy

            if closest_red != None and closest_green != None:
                send_midpoint(closest_red, closest_green)
            
        elif curr_state == State.AVOID_CROWD:
            if state_changed:
                num_left = 10

            closest_red = None
            closest_green = None

            for buoy in buoys:
                if buoy[0] == BuoyType.AVOID_RED:
                    if closest_red is None:
                        closest_red = buoy
                    elif np.linalg.norm(buoy[1] - cur_pos) < np.linalg.norm(closest_red - cur_pos):
                        closest_red = buoy
                elif buoy[0] == BuoyType.AVOID_GREEN:
                    if closest_green is None:
                        closest_green = buoy
                    elif np.linalg.norm(buoy[1] - cur_pos) < np.linalg.norm(closest_green - cur_pos):
                        closest_green = buoy
            
            if closest_red != None and closest_green != None:
                send_midpoint(closest_red, closest_green)

        elif curr_state == State.FIND_SNACK_RUN:
            red_buoy = None
            green_buoy = None
            mark_buoy = None

            for buoy in buoys:
                if buoy[0] == BuoyType.SNACK_RED:
                    red_buoy = buoy
                elif buoy[0] == BuoyType.SNACK_GREEN:
                    green_buoy = buoy
                elif buoy[0] == BuoyType.SNACK_BLUE:
                    mark_buoy = buoy

            if None in [red_buoy, green_buoy]:
                pass
                # TODO use current pos and current heading to somehow get a point to go to the right to explore and try to find the red and green buoys
            else:
                curr_state = State.SNACK_RUN

        elif curr_state == State.SNACK_RUN:
            if state_changed:
                num_left = 7

            red_buoy = None
            green_buoy = None
            mark_buoy = None

            for buoy in buoys:
                if buoy[0] == BuoyType.SNACK_RED:
                    red_buoy = buoy
                elif buoy[0] == BuoyType.SNACK_GREEN:
                    green_buoy = buoy
                elif buoy[0] == BuoyType.SNACK_BLUE:
                    mark_buoy = buoy

            if red_buoy != None and green_buoy != None:
                midpoint = (red_buoy + green_buoy) / 2.0
            else:
                midpoint = snack_run_prev_midpoint
            snack_run_prev_midpoint = midpoint

            if mark_buoy == None:
                if snack_run_prev_marker == None:
                    # estimate the position of the mark buoy
                    mark_buoy = midpoint - np.array([15, 0])
                else:
                    mark_buoy = snack_run_prev_marker
            else:
                snack_run_prev_marker = mark_buoy

            positions = []
            directions = []

            dir = red_buoy - green_buoy
            dir /= np.linalg.norm(dir)

            first_point = mark_buoy + dir * SNACK_RUN_MARKER_CLEARANCE_DIST

            perp = np.array([-dir[1], dir[0]])
            test_point = mark_buoy + perp
            if np.sign(np.cross(dir, mark_buoy - green_buoy)) != np.sign(np.cross(first_point - mark_buoy, test_point - mark_buoy)):
                perp *= -1

            positions.append(midpoint)
            positions.append(midpoint - perp * SNACK_RUN_BEYOND_GOAL_DIST)
            positions.append(first_point)
            positions.append(mark_buoy + perp * SNACK_RUN_MARKER_CLEARANCE_DIST)
            positions.append(mark_buoy - dir * SNACK_RUN_MARKER_CLEARANCE_DIST)

            positions.append(midpoint)
            positions.append(midpoint - perp * SNACK_RUN_BEYOND_GOAL_DIST)

            for cur_pos, next_pos in zip(positions[:-1], positions[1:]):
                dir = next_pos - cur_pos
                dir /= np.linalg.norm(dir)
                directions.append(dir)
            directions.append(-perp)

            for ix, (point, direction) in enumerate(zip(positions, directions)):
                send_waypoint(point, direction, ix)

        elif curr_state == State.FIND_SEAT:
            pass
        elif curr_state == State.WATER_BLAST:
            pass
        elif curr_state == State.SKEEBALL:
            pass
        elif curr_state == State.RETURN:
            return_buoys = []
            for buoy in buoys:
                if buoy[0] == BuoyType.RETURN:
                    return_buoys.append(buoy)
            
            # send the midpoint of these two buoys as the waypoint with the appropriate heading

        prev_state = curr_state
        rate.sleep()

if __name__ == '__main__':
    main()
