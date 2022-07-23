#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np
import matplotlib.pyplot as plt

from snack_run_helper import red_buoy, green_buoy, mark_buoy, BEYOND_GOAL_DIST, MARKER_CLEARANCE_DIST
from occupancy_helper import line, add_buoys, create_grid, create_empty_grid_data, GRID_WIDTH, GRID_HEIGHT
from geom_helper import create_pose_stamped

reached_mark = False # set to True when the boat passes the mark buoy

def compute_goal():
    dir = red_buoy - green_buoy
    dir = np.array([-dir[1], dir[0]])
    dir /= np.linalg.norm(dir)

    mid = (red_buoy + green_buoy) / 2.0
    if np.sign(np.cross(red_buoy - green_buoy, mark_buoy - mid)) != np.sign(
        np.cross(red_buoy - green_buoy, dir)):
        dir *= -1

    if reached_mark:
        return mid - dir * BEYOND_GOAL_DIST, -dir
    else:
        return mark_buoy + dir * MARKER_CLEARANCE_DIST, (red_buoy - green_buoy) / np.linalg.norm(red_buoy - green_buoy)

if __name__ == '__main__':
    rospy.init_node('snack_run_occupancy')

    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10, latch=True)
    goal_pub = rospy.Publisher('/planner/goal', PoseStamped, queue_size=10, latch=True)

    grid = create_grid()

    def compute_grid():
        print('computing grid...')
        grid_data = create_empty_grid_data()

        add_buoys(grid_data, [green_buoy, red_buoy, mark_buoy])

        if reached_mark:
            line_data = line(green_buoy, mark_buoy)
            grid_data[line_data] = 100
        else:
            line_data = line(red_buoy, mark_buoy)
            grid_data[line_data] = 100

        dir = (red_buoy - green_buoy) / np.linalg.norm(red_buoy - green_buoy)
        red_bound = red_buoy + dir * max(GRID_WIDTH, GRID_HEIGHT)
        green_bound = green_buoy - dir * max(GRID_WIDTH, GRID_HEIGHT)

        red_bound = line(red_buoy, red_bound)
        green_bound = line(green_buoy, green_bound)
        grid_data[red_bound] = 100
        grid_data[green_bound] = 100

        goal_pos, goal_dir = compute_goal()
        goal_pose = create_pose_stamped(goal_pos, goal_dir)

        # plt.imshow(np.flip(grid_data, axis=0), interpolation='nearest')
        # plt.show()

        grid.data = np.ndarray.tolist(np.ndarray.flatten(grid_data))
        map_pub.publish(grid)
        goal_pub.publish(goal_pose)
        print('published grid')

    compute_grid()

    def pos_callback(data):
        pos = np.array([data.point.x, data.point.y])
        global reached_mark
        if not reached_mark:
            dir = red_buoy - green_buoy
            target_sign = np.sign(np.cross(dir, mark_buoy - green_buoy))
            curr_sign = np.sign(np.cross(dir, pos - mark_buoy))

            if curr_sign == target_sign:
                reached_mark = True

        compute_grid()

    pos_sub = rospy.Subscriber('/clicked_point', PointStamped, pos_callback)

    rospy.spin()

