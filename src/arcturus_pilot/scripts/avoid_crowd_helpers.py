import numpy as np
import heapq

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

def compute_goal(red_prev, red_last, green_last):
    mid = (red_last + green_last) / 2.0
    
    dir = green_last - red_last
    dir /= np.linalg.norm(dir)

    perp = np.array([-dir[1], dir[0]])
    test_point = mid + perp

    if np.sign(np.cross(red_last - red_prev, green_last - red_last)) != np.sign(
        np.cross(test_point - red_last, green_last - test_point)):
        perp *= -1
    
    return mid + perp * BEYOND_GOAL_DIST, perp
