#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt

from avoid_crowd_helpers import red_buoys, green_buoys, start_pos, get_opt_order, compute_goal
from occupancy_helper import line, add_buoys, create_grid, create_empty_grid_data
from geom_helper import create_pose_stamped

def add_buoy_fence(grid_data, buoys):
    for curr_buoy, next_buoy in zip(buoys[:-1], buoys[1:]):
        line_data = line(curr_buoy, next_buoy)

        grid_data[line_data] = 100

def add_start_fence(grid_data, red_first, green_first):
    offset = red_first - green_first
    
    red_point = start_pos + offset
    red_segment_dir = red_point - red_first
    fencepost_red = red_first + red_segment_dir * 1.5

    green_point = start_pos - offset
    green_segment_dir = green_point - green_first
    fencepost_green = green_first + green_segment_dir * 1.5

    line_1_data = line(red_first, fencepost_red)
    line_2_data = line(fencepost_red, fencepost_green)
    line_3_data = line(fencepost_green, green_first)

    grid_data[line_1_data] = 100
    grid_data[line_2_data] = 100
    grid_data[line_3_data] = 100

if __name__ == '__main__':
    rospy.init_node('avoid_crowd_occupancy')
    
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10, latch=True)
    goal_pub = rospy.Publisher('/planner/goal', PoseStamped, queue_size=10, latch=True)

    grid = create_grid()

    print('computing grid...')
    red_order = get_opt_order(red_buoys)
    green_order = get_opt_order(green_buoys)

    red_ordered = red_buoys[red_order]
    green_ordered = green_buoys[green_order]

    grid_data = create_empty_grid_data()

    add_buoys(grid_data, red_ordered)
    add_buoys(grid_data, green_ordered)
    add_buoy_fence(grid_data, red_ordered)
    add_buoy_fence(grid_data, green_ordered)

    goal_pos, goal_dir = compute_goal(red_ordered[-2], red_ordered[-1], green_ordered[-1])
    goal_pose = create_pose_stamped(goal_pos, goal_dir)

    add_start_fence(grid_data, red_ordered[0], green_ordered[0])

    plt.imshow(np.flip(grid_data, axis=0), interpolation='nearest')
    plt.show()

    grid.data = np.ndarray.tolist(np.ndarray.flatten(grid_data))
    map_pub.publish(grid)
    print('published grid')

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        goal_pub.publish(goal_pose)
        rate.sleep()
