import numpy as np

MARKER_CLEARANCE_DIST = 3 # m
BEYOND_GOAL_DIST = 5 # m

def f2m(x):
    return x * 0.3048

red_buoy = np.array([f2m(50), f2m(50)])
green_buoy = np.array([f2m(50), f2m(56)])
mark_buoy = np.array([f2m(120), f2m(53)])


