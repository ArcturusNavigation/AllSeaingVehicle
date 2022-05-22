#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Quaternion
import tf
import math
import numpy as np
import skimage

GRID_RESOLUTION = 0.2 # m / cell
GRID_WIDTH = 200 # m
GRID_HEIGHT = 150 # m

GRID_NUM_ROWS = int(math.ceil(GRID_HEIGHT / GRID_RESOLUTION))
GRID_NUM_COLS = int(math.ceil(GRID_WIDTH / GRID_RESOLUTION))

BUOY_RADIUS = 0.5 # m

def convert(x):
    return int(round(x / GRID_RESOLUTION))


def line(a, b):
    return skimage.draw.polygon_perimeter(
        [convert(a[1]), convert(b[1]), convert(a[1])],
        [convert(a[0]), convert(b[0]), convert(a[0])],
        shape=(GRID_NUM_ROWS, GRID_NUM_COLS)
    )

def add_buoys(grid_data, buoys):
    for buoy in buoys:
        buoy_data = skimage.draw.ellipse(convert(buoy[1]), convert(buoy[0]), 
            BUOY_RADIUS, BUOY_RADIUS, shape=(GRID_NUM_ROWS, GRID_NUM_COLS))
        grid_data[buoy_data] = 100

def create_grid():
    grid = OccupancyGrid()
    grid.header.stamp = rospy.Time.now()
    grid.header.frame_id = '/map'

    grid.info.map_load_time = rospy.Time.now()
    grid.info.resolution = GRID_RESOLUTION
    grid.info.width = int(GRID_WIDTH / GRID_RESOLUTION)
    grid.info.height = int(GRID_HEIGHT / GRID_RESOLUTION)
    grid.info.origin.position = Point(0, 0, 0)
    grid.info.origin.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))

    return grid

def create_empty_grid_data():
    grid_data = np.zeros((GRID_NUM_ROWS, GRID_NUM_COLS))
    return grid_data
