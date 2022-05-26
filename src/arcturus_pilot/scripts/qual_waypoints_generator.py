#!/usr/bin/env python

import rospy
import numpy as np
from enum import Enum
from arcturus_pilot.msg import Waypoint, WaypointReached
from buoy_types import BuoyType
from geometry_msgs.msg import PoseStamped
from geom_helper import angle_from_dir, sort_buoys_by_dir
from tf import euler_from_quaternion

class State(Enum):
    CHANNEL = 0,
    AVOID_CROWD = 1,
    FIND_SNACK_RUN = 2,
    SNACK_RUN = 3,
    FIND_SEAT = 4,
    WATER_BLAST = 5,
    SKEEBALL = 6,
    RETURN = 7,
    FINISHED = 8

WAYPOINT_DIST_THRESHOLD = 1
SNACK_RUN_MARKER_CLEARANCE_DIST = 1
SNACK_RUN_BEYOND_GOAL_DIST = 3
EXPLORE_DIST = 3
RETURN_DIST = 2

def get_next_state(curr_state):
    """
    Returns what our next state should be depending after we finish the current state
    This basically defines the path that we take through the tasks
    """
    if curr_state == State.CHANNEL:
        return State.AVOID_CROWD
    elif curr_state == State.AVOID_CROWD:
        return State.FIND_SNACK_RUN
    elif curr_state == State.FIND_SNACK_RUN:
        return State.SNACK_RUN
    elif curr_state == State.SNACK_RUN:
        return State.FIND_SEAT
    elif curr_state == State.FIND_SEAT:
        return State.WATER_BLAST
    elif curr_state == State.WATER_BLAST:
        return State.SKEEBALL
    elif curr_state == State.SKEEBALL:
        return State.RETURN
    elif curr_state == State.RETURN:
        return State.FINISHED

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

    cur_buoys = []
    waypoints_sent = []
    curr_state = State.CHANNEL
    prev_state = None

    def buoys_callback(data):
        nonlocal cur_buoys
        visible_buoys = data.buoys
        prev_buoys = cur_buoys
        cur_buoys = visible_buoys
        for prev_buoy in prev_buoys:
            matched = False
            for cur_buoy in visible_buoys:
                if prev_buoy[0] == cur_buoy[0] and np.linalg.norm(prev_buoy, cur_buoy) < WAYPOINT_DIST_THRESHOLD:
                    matched = True
                    break
            if not matched:
                cur_buoys.append(prev_buoy)

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
        state_changed = curr_state != prev_state
        buoys = np.array(cur_buoys)

        if curr_state == State.CHANNEL:
            red_buoys = []
            green_buoys = []

            for buoy in buoys:
                if buoy[0] == BuoyType.CHANNEL_RED:
                    red_buoys.append(buoy[1])
                elif buoy[0] == BuoyType.CHANNEL_GREEN:
                    green_buoys.append(buoy[1])

            red_buoys = sorted(red_buoys, key=lambda buoy: np.linalg.norm(buoy - curr_pos))
            green_buoys = sorted(green_buoys, key=lambda buoy: np.linalg.norm(buoy - curr_pos))

            if len(red_buoys) >= 1 and len(green_buoys) >= 1:
                send_midpoint(red_buoys[0], green_buoys[0], 'channel0')
            if len(red_buoys) >= 2 and len(green_buoys) >= 2:
                send_midpoint(red_buoys[1], green_buoys[1], 'channel1')

            if last_waypoint_visited != -1 and waypoints_sent[last_waypoint_visited][1] == 'channel1':
                curr_state = get_next_state(State.CHANNEL)
            
        elif curr_state == State.AVOID_CROWD:
            red_buoys = []
            green_buoys = []

            for buoy in buoys:
                if buoy[0] == BuoyType.AVOID_RED:
                    red_buoys.append(buoy[1])
                elif buoy[0] == BuoyType.AVOID_GREEN:
                    green_buoys.append(buoy[1])
            
            sorted_red = sort_buoys_by_dir(np.array(red_buoys))
            sorted_green = sort_buoys_by_dir(np.array(green_buoys))

            num_waypoints = min(len(sorted_red), len(sorted_green))
            for i in range(num_waypoints):
                send_midpoint(sorted_red[i], sorted_green[i], 'avoid_crowd' + str(i))

            if last_waypoint_visited != -1 and waypoints_sent[last_waypoint_visited][1] == 'avoid_crowd' + str(num_waypoints - 1):
                curr_state = get_next_state(State.AVOID_CROWD)

        elif curr_state == State.FIND_SNACK_RUN:
            red_buoy = None
            green_buoy = None
            mark_buoy = None

            for buoy in buoys:
                if buoy[0] == BuoyType.SNACK_RED:
                    red_buoy = buoy[1]
                elif buoy[0] == BuoyType.SNACK_GREEN:
                    green_buoy = buoy[1]
                elif buoy[0] == BuoyType.SNACK_BLUE:
                    mark_buoy = buoy[1]

            # TODO: refine this code so that it stores the position at first, goes to to the next position, and doesn't recalculate until it reaches the next
            if None in [red_buoy, green_buoy]:
                right_heading = curr_heading + np.pi / 4
                target_pos = [curr_pos[0] + np.cos(right_heading) * EXPLORE_DIST, curr_pos[1] + np.sin(right_heading) * EXPLORE_DIST]
                send_waypoint(target_pos, right_heading, 'find_snack_run', direction_is_angle=False)
            else:
                curr_state = get_next_state(State.FIND_SNACK_RUN)

        elif curr_state == State.SNACK_RUN:
            red_buoy = None
            green_buoy = None
            mark_buoy = None

            for buoy in buoys:
                if buoy[0] == BuoyType.SNACK_RED:
                    red_buoy = buoy[1]
                elif buoy[0] == BuoyType.SNACK_GREEN:
                    green_buoy = buoy[1]
                elif buoy[0] == BuoyType.SNACK_BLUE:
                    mark_buoy = buoy[1]

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
                curr_state = get_next_state(State.SNACK_RUN)

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
                    return_buoys.append(buoy[1])

            send_midpoint(return_buoys[0], return_buoys[1], 'return')

            if last_waypoint_visited != -1 and waypoints_sent[last_waypoint_visited][1] == 'return':
                curr_state = get_next_state(State.RETURN)

        prev_state = curr_state
        rate.sleep()

if __name__ == '__main__':
    main()
