#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Quaternion, PoseStamped
import tf
import numpy as np
import matplotlib.pyplot as plt
import heapq
import math
import skimage

GRID_RESOLUTION = 0.2 # m / cell
GRID_WIDTH = 200 # m
GRID_HEIGHT = 150 # m

GRID_NUM_ROWS = int(math.ceil(GRID_HEIGHT / GRID_RESOLUTION))
GRID_NUM_COLS = int(math.ceil(GRID_WIDTH / GRID_RESOLUTION))

BUOY_RADIUS = 0.5 # m
BEYOND_GOAL_DIST = 5 # m

# feet to meters
def f2m(x):
    return x * 0.3048

test_set_1 = [
    np.array([( 6.0, 14.0), (14.0, 36.0), (22.0, 60.0), (38.0, 82.0), 
        (75.0, 88.0), (97.0, 64.0), (88.0, 30.0), (80.0, 14.0)]),
    np.array([(26.0,  8.0), (38.0, 37.0), (46.0, 55.0), (54.0, 67.0), 
        (70.0, 72.0), (82.0, 56.0), (70.0, 44.0), (62.0, 20.0)])
]

test_set_2 = [
    np.array([
        (382.0, 353.0), (333.0, 353.0), (310.0, 341.0), (286.0, 320.0), (262.0, 316.0), (235.0, 323.0), (207.0, 330.0), (183.0, 350.0), (160.0, 350.0), (135.0, 335.0), (110.0, 320)
    ]),
    np.array([
        (382.0, 373.0), (333.0, 373.0), (310.0, 361.0), (286.0, 340.0), (262.0, 336.0), (235.0, 343.0), (207.0, 350.0), (183.0, 370.0), (160.0, 370.0), (135.0, 355.0), (110.0, 340)
    ])
]

red_buoys   = test_set_2[0]
green_buoys = test_set_2[1]

np.random.shuffle(red_buoys)
np.random.shuffle(green_buoys)

for i in range(len(red_buoys)):
    red_buoys[i] = (red_buoys[i][0] + np.random.normal(0, 0, 1), red_buoys[i][1] + np.random.normal(0, 0, 1))
    green_buoys[i] = (green_buoys[i][0] + np.random.normal(0, 0, 1), green_buoys[i][1] + np.random.normal(0, 0, 1))

    red_buoys[i] = (f2m(red_buoys[i][0]), f2m(red_buoys[i][1]))
    green_buoys[i] = (f2m(green_buoys[i][0]), f2m(green_buoys[i][1]))

print(red_buoys)
print(green_buoys)

start_pos = np.array((f2m(420.0), f2m(370.0)))

# finds the order of the buoys that minimizes the path distance
# runs in O(V^2 * 2^V * log(V * 2^V)) time
# https://www.baeldung.com/cs/shortest-path-visiting-all-nodes
def get_opt_order(points):
    n = len(points)

    cost = [[np.inf for _ in range(1 << n)] for _ in range(n)]
    parent = [[(-1, -1) for _ in range(1 << n)] for _ in range(n)]
    priority_queue = []

    start = -1
    start_dist = np.inf

    for i in range(n):
        dist = np.linalg.norm(points[i] - start_pos)
        if dist < start_dist:
            start_dist = dist
            start = i

    priority_queue.append((0, (start, 1 << start)))
    cost[start][1 << start] = 0

    heapq.heapify(priority_queue)

    while priority_queue:
        curr_cost, (curr_node, curr_bitmask) = heapq.heappop(priority_queue)
        if curr_cost - cost[curr_node][curr_bitmask] > 0.001:
            continue
        for other_node in range(n):
            dist = np.linalg.norm(points[curr_node] - points[other_node])
            other_bitmask = curr_bitmask | (1 << other_node)
            if cost[other_node][other_bitmask] > curr_cost + dist:
                cost[other_node][other_bitmask] = curr_cost + dist
                parent[other_node][other_bitmask] = (curr_node, curr_bitmask)
                heapq.heappush(priority_queue, (curr_cost + dist, (other_node, other_bitmask)))
    
    end_node = -1
    end_node_dist = np.inf
    all_mask = (1 << n) - 1

    for i in range(n):
        if cost[i][all_mask] < end_node_dist:
            end_node = i
            end_node_dist = cost[i][all_mask]
    
    order = []
    curr_node = end_node
    curr_mask = all_mask
    while curr_node != start:
        order = [curr_node] + order
        curr_node, curr_mask = parent[curr_node][curr_mask]

    return [start] + order

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

    for curr_buoy, next_buoy in zip(buoys[:-1], buoys[1:]):
        line_data = line(curr_buoy, next_buoy)

        grid_data[line_data] = 100

def compute_goal(red_prev, red_last, green_last):
    mid = (red_last + green_last) / 2.0
    
    dir = green_last - red_last
    dir /= np.linalg.norm(dir)

    perp = np.array([-dir[1], dir[0]])
    test_point = mid + perp

    if np.sign(np.cross(red_last - red_prev, green_last - red_last)) != np.sign(
        np.cross(test_point - red_last, green_last - test_point)):
        perp *= -1
    
    return mid + perp * BEYOND_GOAL_DIST

def add_start_fence(grid_data, red_first, red_second, green_first, green_second):
    dist = np.linalg.norm(red_first - green_first)
    
    red_segment_dir = red_first - red_second
    red_segment_dir /= np.linalg.norm(red_segment_dir)

    green_segment_dir = green_first - green_second
    green_segment_dir /= np.linalg.norm(green_segment_dir)

    fencepost_red = red_first + red_segment_dir * dist
    fencepost_green = green_first + green_segment_dir * dist

    line_1_data = line(red_first, fencepost_red)
    line_2_data = line(fencepost_red, fencepost_green)
    line_3_data = line(fencepost_green, green_first)

    grid_data[line_1_data] = 100
    grid_data[line_2_data] = 100
    grid_data[line_3_data] = 100

if __name__ == '__main__':
    rospy.init_node('compute_occupancy_grid')
    
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10, latch=True)
    goal_pub = rospy.Publisher('/planner/goal', PoseStamped, queue_size=10, latch=True)

    grid = OccupancyGrid()
    grid.header.stamp = rospy.Time.now()
    grid.header.frame_id = '/map'

    grid.info.map_load_time = rospy.Time.now()
    grid.info.resolution = GRID_RESOLUTION
    grid.info.width = int(GRID_WIDTH / GRID_RESOLUTION)
    grid.info.height = int(GRID_HEIGHT / GRID_RESOLUTION)
    grid.info.origin.position = Point(0, 0, 0)
    grid.info.origin.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))

    print('computing grid...')
    red_order = get_opt_order(red_buoys)
    green_order = get_opt_order(green_buoys)

    red_ordered = red_buoys[red_order]
    green_ordered = green_buoys[green_order]

    grid_data = np.zeros((GRID_NUM_ROWS, GRID_NUM_COLS))

    add_buoys(grid_data, red_ordered)
    add_buoys(grid_data, green_ordered)

    goal = compute_goal(red_ordered[-2], red_ordered[-1], green_ordered[-1])
    goal_pose = PoseStamped()
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position = Point(x = goal[0], y = goal[1], z = 0)
    # TODO compute orientation for this
    goal_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))

    add_start_fence(grid_data, red_ordered[0], red_ordered[1], green_ordered[0], green_ordered[1])

    plt.imshow(np.flip(grid_data, axis=0), interpolation='nearest')
    plt.show()

    grid.data = np.ndarray.tolist(np.ndarray.flatten(grid_data))
    map_pub.publish(grid)
    print('published grid')

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        goal_pub.publish(goal_pose)
        rate.sleep()
