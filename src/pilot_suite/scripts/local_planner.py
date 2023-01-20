<<<<<<< HEAD:src/arcturus_pilot/scripts/local_planner.py
#!/usr/bin/env python

import rospy
from arcturus_pilot.msg import RawWaypoint, ProcessedWaypoint, WaypointReached
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion
from geom_helper import angle_from_dir
import numpy as np
import skimage

SEARCH_DIST = 0.125

class LocalPlanner():
    def __init__(self):
        self.raw_waypoint_sub = rospy.Subscriber('arcturus_pilot/raw_waypoint', RawWaypoint, self.raw_waypoint_callback)
        self.processed_waypoint_pub = rospy.Publisher('arcturus_pilot/processed_waypoint', ProcessedWaypoint, queue_size=10)
        self.reached_waypoint_sub = rospy.Subscriber('arcturus_pilot/waypoint_reached', WaypointReached, self.reached_waypoint_callback)
        
        self.pose_sub = rospy.Subscriber('arcturus_pilot/pose', PoseStamped, self.pose_callback)
        self.occupancy_sub = rospy.Subscriber('/occupancy_grid', OccupancyGrid, self.occupancy_callback)

        self.occupancy = None
        self.occupancy_cols = 0
        self.occupancy_rows = 0
        self.occupancy_resolution = 0
        self.pos = np.array([0, 0])
        self.heading = 0
        self.order = -1

    def run(self):
        rospy.spin()

    def convert(self, x):
        return int(round(x / self.occupancy_resolution))

    def find_obstacle(self, target):
        x = target[0]
        y = target[1]

        # draw line from current pos to (x, y) and see if it intersects with anything on the occupancy grid
        rr, cc = skimage.draw.polygon_perimeter(
            [self.convert(self.pos[1]), self.convert(y), self.convert(self.pos[1])],
            [self.convert(self.pos[0]), self.convert(x), self.convert(self.pos[0])],
            shape=(self.occupancy_rows, self.occupancy_cols)
        )

        rr = rr[:rr.shape[0] // 2]
        cc = cc[:cc.shape[0] // 2]

        cur_pos_np = np.array(self.pos)

        obstacle = None
        for r, c in zip(rr, cc):
            line_pos = np.array([c, r]) * self.occupancy_resolution
            if self.occupancy[r, c] > 0:
                if obstacle is None:
                    obstacle = line_pos
                else:
                    if np.linalg.norm(line_pos - cur_pos_np) < \
                        np.linalg.norm(obstacle - cur_pos_np):
                        obstacle = line_pos

        return obstacle

    def adjust_point_side(self, point, dir):
        perp = np.array([dir[1], -dir[0]])

        valid_waypoint = None

        for i in range(1, 100):
            test_1 = point + perp * i * SEARCH_DIST
            test_2 = point - perp * i * SEARCH_DIST

            if self.occupancy[self.convert(test_1[1]), self.convert(test_1[0])] == 0:
                valid_waypoint = test_1
                break
            elif self.occupancy[self.convert(test_2[1]), self.convert(test_2[0])] == 0:
                valid_waypoint = test_2
                break
        
        if valid_waypoint is None:
            rospy.logerr("Could not find valid waypoint")
            return point

        return valid_waypoint

    def adjust_point_back(self, point, dir):
        valid_waypoint = None

        for i in range(100):
            test = point - dir * i * SEARCH_DIST

            if self.occupancy[self.convert(test[1]), self.convert(test[0])] == 0:
                valid_waypoint = test
                break
        
        if valid_waypoint is None:
            rospy.logerr("Could not find valid waypoint")
            return point

        return valid_waypoint

    def raw_waypoint_callback(self, data):
        (x, y, heading, order) = (data.x, data.y, data.heading, data.order)

        # if the target is not our next immediate point, don't process it
        if self.order + 1 != order or self.occupancy is None:
            self.processed_waypoint_pub.publish(ProcessedWaypoint(x, y, heading, order, False))
            return

        target = np.array([x, y])

        target_dir = target - self.pos
        target_dir /= np.linalg.norm(target_dir)

        # if the target is inside of an obstacle, stop at the earliest point before the obstacle
        target = self.adjust_point_back(target, target_dir)

        # check if the line between boat and target goes through any obstacles
        obstacle = self.find_obstacle(target)

        if obstacle is None:
            self.processed_waypoint_pub.publish(ProcessedWaypoint(x, y, heading, order, False))
            return
        
        # create a temporary waypoint to the side of the obstacle
        valid_waypoint = self.adjust_point_side(obstacle, target_dir)
        
        dir = target - valid_waypoint
        dir /= np.linalg.norm(dir)
        heading = angle_from_dir(dir)
        self.processed_waypoint_pub.publish(ProcessedWaypoint(valid_waypoint[0], valid_waypoint[1], heading, order, True))

    def reached_waypoint_callback(self, data):
        self.order = data.order

    def pose_callback(self, data):
        self.pos[0] = data.pose.position.x
        self.pos[1] = data.pose.position.y
        explicit_quat = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        self.heading = euler_from_quaternion(explicit_quat)[2]

    def occupancy_callback(self, data):
        self.occupancy_cols = data.info.width
        self.occupancy_rows = data.info.height
        self.occupancy_resolution = data.info.resolution
        self.occupancy = np.array(data.data).reshape((self.occupancy_rows, self.occupancy_cols))

if __name__ == '__main__':
    rospy.init_node('local_planner')
    local_planner = LocalPlanner()
=======
#!/usr/bin/env python

import rospy
from pilot_suite.msg import RawWaypoint, ProcessedWaypoint, WaypointReached
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion
from pilot_suite.geom_utils import angle_from_dir
import numpy as np
import skimage

SEARCH_DIST_STEP = 0.125
SEARCH_DIST_START = 0.25

class LocalPlanner():
    def __init__(self):
        self.raw_waypoint_sub = rospy.Subscriber('pilot_suite/raw_waypoint', RawWaypoint, self.raw_waypoint_callback)
        self.processed_waypoint_pub = rospy.Publisher('pilot_suite/processed_waypoint', ProcessedWaypoint, queue_size=10)
        self.reached_waypoint_sub = rospy.Subscriber('pilot_suite/waypoint_reached', WaypointReached, self.reached_waypoint_callback)

        self.debug_raw_pub = rospy.Publisher('pilot_suite/debug_raw', PointStamped, queue_size=5)
        self.debug_processed_pub = rospy.Publisher('pilot_suite/debug_processed', PointStamped, queue_size=5)

        self.pose_sub = rospy.Subscriber('pilot_suite/pose', PoseStamped, self.pose_callback)
        self.occupancy_sub = rospy.Subscriber('/occupancy_grid', OccupancyGrid, self.occupancy_callback)

        self.occupancy = None
        self.occupancy_cols = 0
        self.occupancy_rows = 0
        self.occupancy_resolution = 0
        self.pos = np.array([0, 0])
        self.heading = 0
        self.order = -1

    def run(self):
        rospy.spin()

    def convert(self, x):
        return int(round(x / self.occupancy_resolution))

    def find_obstacle(self, target):
        x = target[0]
        y = target[1]

        # draw line from current pos to (x, y) and see if it intersects with anything on the occupancy grid
        rr, cc = skimage.draw.polygon_perimeter(
            [self.convert(self.pos[1]), self.convert(y), self.convert(self.pos[1])],
            [self.convert(self.pos[0]), self.convert(x), self.convert(self.pos[0])],
            shape=(self.occupancy_rows, self.occupancy_cols)
        )

        rr = rr[:rr.shape[0] // 2]
        cc = cc[:cc.shape[0] // 2]

        cur_pos_np = np.array(self.pos)

        obstacle_shortest = None
        obstacle_farthest = None
        for r, c in zip(rr, cc):
            line_pos = np.array([c, r]) * self.occupancy_resolution
            if self.occupancy[r, c] > 50:
                if obstacle_shortest is None:
                    obstacle_shortest = line_pos
                    obstacle_farthest = line_pos
                else:
                    if np.linalg.norm(line_pos - cur_pos_np) < \
                        np.linalg.norm(obstacle_shortest - cur_pos_np):
                        obstacle_shortest = line_pos

                    if np.linalg.norm(line_pos - cur_pos_np) > \
                        np.linalg.norm(obstacle_farthest - cur_pos_np):
                        obstacle_farthest = line_pos

        if obstacle_shortest is None:
            return None

        return (obstacle_shortest + obstacle_farthest) / 2.0

    def adjust_point_side(self, point, dir):
        perp = np.array([dir[1], -dir[0]])

        valid_waypoint = None

        for i in range(100):
            test_1 = point + perp * (i * SEARCH_DIST_STEP + SEARCH_DIST_START)
            test_2 = point - perp * (i * SEARCH_DIST_STEP + SEARCH_DIST_START)

            if self.occupancy[self.convert(test_1[1]), self.convert(test_1[0])] == 0:
                valid_waypoint = test_1
                break
            elif self.occupancy[self.convert(test_2[1]), self.convert(test_2[0])] == 0:
                valid_waypoint = test_2
                break
        
        if valid_waypoint is None:
            rospy.logerr("Could not find valid waypoint")
            return point

        return valid_waypoint

    def adjust_point_back(self, point, dir):
        valid_waypoint = None

        for i in range(100):
            test = point - dir * ((i - 1) * SEARCH_DIST_STEP + SEARCH_DIST_START if i != 0 else 0)

            if self.occupancy[self.convert(test[1]), self.convert(test[0])] == 0:
                valid_waypoint = test
                break
        
        if valid_waypoint is None:
            rospy.logerr("Could not find valid waypoint")
            return point

        return valid_waypoint

    def raw_waypoint_callback(self, data):
        (x, y, heading, order) = (data.x, data.y, data.heading, data.order)

        # if the target is not our next immediate point, don't process it
        if self.order + 1 != order or self.occupancy is None or data.ignore_local_planner:
            self.processed_waypoint_pub.publish(ProcessedWaypoint(x, y, heading, order, False))
            return

        self.debug_raw_pub.publish(PointStamped(Header(0, rospy.Time.now(), '/map'), Point(x, y, 0)))

        target = np.array([x, y])

        target_dir = target - self.pos
        target_dir /= np.linalg.norm(target_dir)

        target = self.adjust_point_back(target, target_dir)
        self.processed_waypoint_pub.publish(ProcessedWaypoint(target[0], target[1], heading, order, False))
        self.debug_processed_pub.publish(PointStamped(Header(0, rospy.Time.now(), '/map'), Point(target[0], target[1], 0)))

        # check if the line between boat and target goes through any obstacles
        obstacle = self.find_obstacle(target)

        if obstacle is None:
            return
        
        # create a temporary waypoint to the side of the obstacle
        valid_waypoint = self.adjust_point_side(obstacle, target_dir)

        self.debug_raw_pub.publish(PointStamped(Header(0, rospy.Time.now(), '/map'), Point(obstacle[0], obstacle[1], 0)))
        
        dir = target - valid_waypoint
        dir /= np.linalg.norm(dir)
        heading = angle_from_dir(dir)
        self.processed_waypoint_pub.publish(ProcessedWaypoint(valid_waypoint[0], valid_waypoint[1], heading, order, True))
        self.debug_processed_pub.publish(PointStamped(Header(0, rospy.Time.now(), '/map'), Point(valid_waypoint[0], valid_waypoint[1], 0)))

    def reached_waypoint_callback(self, data):
        self.order = data.order

    def pose_callback(self, data):
        self.pos[0] = data.pose.position.x
        self.pos[1] = data.pose.position.y
        explicit_quat = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        self.heading = euler_from_quaternion(explicit_quat)[2]

    def occupancy_callback(self, data):
        self.occupancy_cols = data.info.width
        self.occupancy_rows = data.info.height
        self.occupancy_resolution = data.info.resolution
        self.occupancy = np.array(data.data).reshape((self.occupancy_rows, self.occupancy_cols))

if __name__ == '__main__':
    rospy.init_node('local_planner')
    local_planner = LocalPlanner()
>>>>>>> 351031eaa37bdb673668a5cb8e1ee8d050ae4dba:src/pilot_suite/scripts/local_planner.py
    local_planner.run()