#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
import numpy as np

from avoid_crowd_helpers import red_buoys, green_buoys, get_opt_order, compute_goal
from path_helper import create_path
from geom_helper import create_pose_stamped

if __name__ == '__main__':
    rospy.init_node('avoid_crowd_waypoints')
    
    path_pub = rospy.Publisher('/avoid_crowd_path', Path, queue_size=10, latch=True)

    red_order = get_opt_order(red_buoys)
    green_order = get_opt_order(green_buoys)

    red_ordered = red_buoys[red_order]
    green_ordered = green_buoys[green_order]

    path_data = create_path()
    path_data.poses = []
    goal_pos, goal_dir = compute_goal(red_ordered[-2], red_ordered[-1], green_ordered[-1])

    positions = []
    directions = []

    for red_buoy, green_buoy in zip(red_ordered, green_ordered):
        target_pos = (red_buoy + green_buoy) / 2
        positions.append(target_pos)
    
    positions.append(goal_pos)

    for pos, next_pos in zip(positions[:-1], positions[1:]):
        dir = next_pos - pos
        dir /= np.linalg.norm(dir)
        directions.append(dir)

    for pos, dir in zip(positions[:-1], directions + directions):
        path_data.poses.append(create_pose_stamped(pos, dir))
    path_data.poses.append(create_pose_stamped(goal_pos, goal_dir))

    path_pub.publish(path_data)

    print('Published path')

    rospy.spin()
