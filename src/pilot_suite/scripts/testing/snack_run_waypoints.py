#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
import numpy as np

from snack_run_helper import red_buoy, green_buoy, mark_buoy, MARKER_CLEARANCE_DIST, BEYOND_GOAL_DIST
from path_helper import create_path
<<<<<<< HEAD:src/arcturus_pilot/scripts/testing/snack_run_waypoints.py
from geom_helper import create_pose_stamped
=======
from AllSeaingVehicle.src.pilot_suite.scripts.utils.geom_helper import create_pose_stamped
>>>>>>> 351031eaa37bdb673668a5cb8e1ee8d050ae4dba:src/pilot_suite/scripts/testing/snack_run_waypoints.py

if __name__ == '__main__':
    rospy.init_node('snack_run_waypoints')
    
    path_pub = rospy.Publisher('/snack_run_path', Path, queue_size=10, latch=True)

    path_data = create_path()
    path_data.poses = []
    
    positions = []
    directions = []

    positions.append((red_buoy + green_buoy) / 2.0)
    dir = red_buoy - green_buoy
    dir /= np.linalg.norm(dir)

    first_point = mark_buoy + dir * MARKER_CLEARANCE_DIST

    perp = np.array([-dir[1], dir[0]])
    test_point = mark_buoy + perp
    if np.sign(np.cross(dir, mark_buoy - green_buoy)) != np.sign(np.cross(first_point - mark_buoy, test_point - mark_buoy)):
        perp *= -1

    positions.append(first_point)
    positions.append(mark_buoy + perp * MARKER_CLEARANCE_DIST)
    positions.append(mark_buoy - dir * MARKER_CLEARANCE_DIST)

    positions.append((red_buoy + green_buoy) / 2.0)
    positions.append((red_buoy + green_buoy) / 2.0 - perp * BEYOND_GOAL_DIST)

    for pos, next_pos in zip(positions[:-1], positions[1:]):
        dir = next_pos - pos
        dir /= np.linalg.norm(dir)
        path_data.poses.append(create_pose_stamped(pos, dir))
    path_data.poses.append(create_pose_stamped(positions[-1], -perp))

    path_pub.publish(path_data)

    rospy.spin()
