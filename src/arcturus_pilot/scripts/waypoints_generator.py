#!/usr/bin/env python

import rospy
import numpy as np
from enum import Enum
from arcturus_pilot.msg import RawWaypoint, WaypointReached
from object_types import ObjectType
from geometry_msgs.msg import PoseStamped
from geom_helper import angle_from_dir, sort_buoys_by_dir
from tf import euler_from_quaternion

from arcturus_pilot.msg import ObjectList

# TODO: investigate what happens when you try to send a waypoint to the boat that is behind it with the same heading
# as the current boat. If the boat just tries to go backwards, then have to adjust the local planner code so that the
# intermediate waypoints have the correct heading. If not, then have to redo the code for generating waypoints to get
# out of dock

BUOY_DIST_THRESHOLD = 1 # distance buoys have to be from each other to be considered the same buoy
SNACK_RUN_MARKER_CLEARANCE_DIST = 1 # distance the boat will attempt to go past the snack run marker
SNACK_RUN_BEYOND_GOAL_DIST = 3 # distance the boat will go past the snack run opening / closing


SEARCH_EXPLORE_DIST = 5 # for the linear search behaviors, how far in a direction the search will go
SEARCH_TURN_AROUND_RADIUS = 1 # radius of the circle used for turning around
SEARCH_CIRCLE_RADIUS = 2 # radius of the circle in the search circle behavior
SEARCH_TIMEOUT = 15 # seconds the boat will spend searching before giving up and skipping the task

DOCK_TASKS_CLEARING_DIST = 3.0 # how far away from the dock task the boat will go before it stops to line up with the dock
# direction vectors for the dock tasks that give the direction the boat should face to complete them 

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
    FORWARD = 2,
    CIRCLE = 3,
    TURN_AROUND = 4,
    SKIP = 5

class SearchData():
    def __init__(self, next_state, search_behavior, init_pos, init_heading):
        self.next_state = next_state
        self.search_behavior = search_behavior
        self.init_pos = init_pos
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

        elif self.search_behavior == SearchBehavior.TURN_AROUND:
            target_heading_1 = self.init_heading + np.pi / 2
            dir_1 = self.init_heading + np.pi / 4
            target_pos_1 = self.init_pos + np.array([np.cos(dir_1), np.sin(dir_1)]) * SEARCH_TURN_AROUND_RADIUS
            waypoints.append((target_pos_1, target_heading_1, waypoints_id_prefix + '0', False))

            target_heading_2 = self.init_heading + np.pi
            dir_2 = self.init_heading + np.pi / 2
            target_pos_2 = self.init_pos + np.array([np.cos(dir_2), np.sin(dir_2)]) * SEARCH_TURN_AROUND_RADIUS
            waypoints.append((target_pos_2, target_heading_2, waypoints_id_prefix + '1', False))

        return waypoints

"""
Generates raw waypoints based on the current task
Sends these raw waypoints to the arcturus_pilot/local_planner node, which processes them and then sends them to
arcturus_pilot/waypoint_pilot

Receives data from the sensor suite with the locations of all of the buoys (expected in the local_origin frame)
All waypoints are sent in the local_origin frame
"""
def main():
    rospy.init_node('waypoints_generator')
    
    curr_pos = np.array([0, 0])
    curr_heading = 0
    last_waypoint_visited = -1

    curr_objects = []
    waypoints_sent = []
    curr_state = None    
    search_data = None

    waypoint_pub = rospy.publisher('arcturus_pilot/raw_waypoint', RawWaypoint, queue_size=10)
    reached_waypoint_sub = rospy.Subscriber('arcturus_pilot/waypoint_reached', WaypointReached, reached_waypoint_callback)
    pose_sub = rospy.Subscriber('arcturus_pilot/pose', PoseStamped, pose_callback)
    objects_sub = rospy.Subscriber('sensor_suite/objects', ObjectList, objects_callback)

    def search(next_state, search_behavior=SearchBehavior.SKIP):
        """
        Given the next desired state, sets up searching for the next state 
        """
        nonlocal curr_state
        nonlocal search_data

        curr_state = State.SEARCH
        search_data = SearchData(next_state, search_behavior, curr_pos, curr_heading)
        rospy.Timer(rospy.duration(SEARCH_TIMEOUT), search_timeout, True)

    def search_timeout():
        nonlocal curr_state
        curr_state = search_data.next_state

    def go_next_state():
        """
        Sets up the next state given our current state
        Defines our path through the course
        """
        nonlocal curr_state

        if curr_state == State.CHANNEL:
            search(State.AVOID_CROWD, SearchBehavior.FORWARD)

        elif curr_state == State.AVOID_CROWD:
            search(State.SNACK_RUN, SearchBehavior.STEER_RIGHT)
    
        elif curr_state == State.SNACK_RUN:
            search(State.SKEEBALL, SearchBehavior.FORWARD)

        elif curr_state == State.FIND_SEAT:
            search(State.WATER_BLAST, SearchBehavior.TURN_AROUND)

        elif curr_state == State.WATER_BLAST:
            search(State.SKEEBALL, SearchBehavior.STEER_RIGHT)

        elif curr_state == State.SKEEBALL:
            search(State.RETURN)

        elif curr_state == State.RETURN:
            curr_state = State.FINISHED

    def objects_callback(data):
        nonlocal curr_objects
        visible_buoys = data.buoys
        # prev_buoys = curr_objects
        curr_objects = np.zeros((0, 3))

        for buoy in visible_buoys:
            curr_objects = np.append(curr_objects, [[buoy.type, buoy.x, buoy.y]], axis=0)

        # for prev_buoy in prev_buoys:
        #     matched = False
        #     for cur_buoy in visible_buoys:
        #         if prev_buoy[0] == cur_buoy[0] and np.linalg.norm(prev_buoy - cur_buoy) < BUOY_DIST_THRESHOLD:
        #             matched = True
        #             break
        #     if not matched:
        #         np.append(curr_objects, prev_buoy[np.newaxis, :], axis=0)

    def pose_callback(data):
        nonlocal curr_pos
        nonlocal curr_heading
        curr_pos[0] = data.pose.position.x
        curr_pos[1] = data.pose.position.y
        curr_heading = euler_from_quaternion(data.pose.orientation)[2]

    def reached_waypoint_callback(data):
        nonlocal last_waypoint_visited
        nonlocal curr_state
        last_waypoint_visited = data.order

    def find_index(waypoint_id):
        for ix, waypoint in enumerate(waypoints_sent):
            if waypoint[1] == waypoint_id:
                return ix
        return -1

    def send_waypoint(position, direction, waypoint_id, is_dir_vector=True):
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

        waypoint_pub.publish(RawWaypoint(position[0], position[1], angle_from_dir(direction) if is_dir_vector else direction, ix))
    
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
    

    def is_last_visited(waypoint_id):
        return last_waypoint_visited != -1 and waypoints_sent[last_waypoint_visited][1] == waypoint_id

    # start up initial state
    search(State.CHANNEL, SearchBehavior.FORWARD)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        objects = np.array(curr_objects)

        if curr_state == State.CHANNEL:
            red_buoys = []
            green_buoys = []

            for object in objects:
                if object[0] == ObjectType.CHANNEL_RED:
                    red_buoys.append(object[1:])
                elif object[0] == ObjectType.CHANNEL_GREEN:
                    green_buoys.append(object[1:])

            red_buoys = sorted(red_buoys, key=lambda buoy: np.linalg.norm(buoy - curr_pos))
            green_buoys = sorted(green_buoys, key=lambda buoy: np.linalg.norm(buoy - curr_pos))

            if len(red_buoys) >= 1 and len(green_buoys) >= 1:
                send_midpoint(red_buoys[0], green_buoys[0], 'channel0')
            if len(red_buoys) >= 2 and len(green_buoys) >= 2:
                send_midpoint(red_buoys[1], green_buoys[1], 'channel1')

            if is_last_visited('channel1'):
                go_next_state()
            
        elif curr_state == State.AVOID_CROWD:
            red_buoys = []
            green_buoys = []

            for object in objects:
                if object[0] == ObjectType.AVOID_RED:
                    red_buoys.append(object[1:])
                elif object[0] == ObjectType.AVOID_GREEN:
                    green_buoys.append(object[1:])

            # if we haven't found anything, then just skip (should never happen tbh)
            if len(red_buoys) == 0 or len(green_buoys) == 0:
                go_next_state()
            else:
                sorted_red = sort_buoys_by_dir(np.array(red_buoys))
                sorted_green = sort_buoys_by_dir(np.array(green_buoys))

                num_waypoints = min(len(sorted_red), len(sorted_green))
                for i in range(num_waypoints):
                    send_midpoint(sorted_red[i], sorted_green[i], 'avoid_crowd' + str(i))

                if is_last_visited('avoid_crowd' + str(num_waypoints - 1)):
                    go_next_state()

        elif curr_state == State.SNACK_RUN:
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
            if red_buoy == None and green_buoy == None:
                go_next_state()
            else:
                # estimate the red position using green buoy
                if red_buoy == None:
                    red_buoy = green_buoy - np.array(3.0)

                # estimate the red position using green buoy
                if green_buoy == None:
                    green_buoy = red_buoy + np.array(3.0)

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

                if is_last_visited('snack_run' + str(last_ix)):
                    go_next_state()

        elif curr_state == State.FIND_SEAT:
            find_seat_pos = None

            for object in objects:
                if object[0] == ObjectType.FIND_SEAT:
                    find_seat_pos = object[1:]
            
            if find_seat_pos == None:
                go_next_state()
            else:
                send_waypoint(find_seat_pos - FIND_SEAT_DIR * DOCK_TASKS_CLEARING_DIST, FIND_SEAT_DIR, 'find_seat0')

                # TODO wait for / somehow calculate waypoint for which dock to go to
                find_seat_target = np.array([0, 0])
                send_waypoint(find_seat_target, FIND_SEAT_DIR, 'find_seat1')
                send_waypoint(find_seat_pos - FIND_SEAT_DIR * DOCK_TASKS_CLEARING_DIST, FIND_SEAT_DIR, 'find_seat2')
                    
                if is_last_visited('find_seat2'):
                    go_next_state()

        elif curr_state == State.WATER_BLAST:
            water_blast_pos = None

            for object in objects:
                if object[0] == ObjectType.WATER_BLAST:
                    water_blast_pos = object[1:]
            
            if water_blast_pos == None:
                go_next_state()
            else:
                send_waypoint(water_blast_pos - WATER_BLAST_DIR * DOCK_TASKS_CLEARING_DIST, WATER_BLAST_DIR, 'water_blast0')
                send_waypoint(water_blast_pos, WATER_BLAST_DIR, 'water_blast1')

                if is_last_visited('water_blast1'):
                    # TODO send signal for water blaster to go
                    pass
                
                # TODO receive signal when water blaster is finished
                water_blast_signal = True
                if water_blast_signal:
                    send_waypoint(water_blast_pos - WATER_BLAST_DIR * DOCK_TASKS_CLEARING_DIST, WATER_BLAST_DIR, 'water_blast2')
                    
                if is_last_visited('water_blast2'):
                    go_next_state()

        elif curr_state == State.SKEEBALL:
            skeeball_pos = None

            for object in objects:
                if object[0] == ObjectType.SKEEBALL:
                    skeeball_pos = object[1:]
            
            if skeeball_pos == None:
                go_next_state()
            else:
                send_waypoint(skeeball_pos - SKEEBALL_DIR * DOCK_TASKS_CLEARING_DIST, SKEEBALL_DIR, 'skeeball0')
                send_waypoint(skeeball_pos, SKEEBALL_DIR, 'skeeball1')

                if is_last_visited('skeeball1'):
                    # TODO send signal for skeeball to go
                    pass
                
                # TODO receive signal when skeeball is finished
                skeeball_signal = True
                if skeeball_signal:
                    send_waypoint(skeeball_pos - SKEEBALL_DIR * DOCK_TASKS_CLEARING_DIST, SKEEBALL_DIR, 'skeeball2')
                    
                if is_last_visited('skeeball2'):
                    go_next_state()

        elif curr_state == State.RETURN:
            return_buoys = []
            for object in objects:
                if object[0] == ObjectType.RETURN:
                    return_buoys.append(object[1:])

            if len(return_buoys) < 2:
                go_next_state()
            else:
                send_midpoint(return_buoys[0], return_buoys[1], 'return')

                if is_last_visited('return'):
                    go_next_state()
        elif curr_state == State.SEARCH:
            if search_data.has_found_targets(objects):
                curr_state = search_data.next_state
            else:
                for waypoint_data in search_data.get_waypoints():
                    send_waypoint(*waypoint_data)

        rate.sleep()

if __name__ == '__main__':
    main()
