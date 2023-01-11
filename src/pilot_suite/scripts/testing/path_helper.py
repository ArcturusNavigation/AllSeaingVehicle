#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
import math
import numpy as np

def create_path():
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = '/map'

    return path
