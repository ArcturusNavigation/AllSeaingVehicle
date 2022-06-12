from arcturus_pilot.scripts.testing.occupancy_helper import GRID_RESOLUTION
import rospy
from arcturus_pilot.msg import RawWaypoint, ProcessedWaypoint, WaypointReached
from geometry_msgs.msg import PoseStamped, OccupancyGrid
from tf import euler_from_quaternion
from geom_helper import angle_from_dir
import numpy as np
import skimage

SEARCH_DIST = 0.5

class LocalPlanner():
    def __init__(self):
        self.raw_waypoint_sub = rospy.Subscriber('arcturus_pilot/raw_waypoint', RawWaypoint, self.raw_waypoint_callback)
        self.processed_waypoint_pub = rospy.Publisher('arcturus_pilot/processed_waypoint', ProcessedWaypoint)
        self.reached_waypoint_sub = rospy.Subscriber('arcturus_pilot/waypoint_reached', WaypointReached, self.reached_waypoint_callback)
        self.pose_sub = rospy.Subscriber('arcturus_pilot/pose', PoseStamped, self.pose_callback)
        self.occupancy_sub = rospy.Subscriber('sensor_suite/occupancy', OccupancyGrid, self.occupancy_callback)

        self.occupancy = None
        self.occupancy_width = 0
        self.occupancy_height = 0
        self.occupancy_resolution = 0
        self.pos = np.array([0, 0])
        self.heading = 0
        self.order = -1

    def run():
        rospy.init_node('local_planner')
        rospy.spin()

    def convert(self, x):
        return int(round(x / GRID_RESOLUTION))

    def find_obstacle(self, target):
        x = target[0]
        y = target[1]

        # draw line from current pos to (x, y) and see if it intersects with anything on the occupancy grid
        rr, cc = skimage.draw.polygon_perimeter(
            [self.convert(self.pos[1]), self.convert(y), self.convert(self.pos[1])],
            [self.convert(self.pos[0]), self.convert(x), self.convert(self.pos[0])],
            shape=(self.occupancy_height, self.occupancy_width)
        )

        rr = rr[:rr.shape[0] // 2]
        cc = cc[:cc.shape[0] // 2]

        cur_pos_np = np.array(self.pos)

        obstacle = None
        for r, c in zip(rr, cc):
            line_pos = np.array([c, r]) * self.occupancy_resolution
            if self.occupancy[r, c] > 0:
                if obstacle == None:
                    obstacle = line_pos
                else:
                    if np.linalg.norm(line_pos - cur_pos_np) < \
                        np.linalg.norm(obstacle - cur_pos_np):
                        obstacle = line_pos

        return obstacle

    def adjust_point(self, dir, point):
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

    def raw_waypoint_callback(self, data):
        (x, y, heading, order) = (data.x, data.y, data.heading, data.order)

        # if the target is not our next immediate point, don't process it
        if self.order + 1 != order or self.occupancy is None:
            self.processed_waypoint_pub.publish(ProcessedWaypoint(x, y, heading, order, False))
            return

        target = np.array([x, y])

        target_dir = target - self.pos
        target_dir /= np.linalg.norm(target_dir)

        # if the target is inside of an obstacle, move it out
        target = self.adjust_point(target, target_dir)

        # check if the line between boat and target goes through any obstacles
        obstacle = self.find_obstacle(target)

        if obstacle == None:
            self.processed_waypoint_pub.publish(ProcessedWaypoint(x, y, heading, order, False))
            return
        
        # create a temporary waypoint to the side of the obstacle
        valid_waypoint = self.adjust_point(target, target_dir)
        
        dir = target - valid_waypoint
        dir /= np.linalg.norm(dir)
        heading = angle_from_dir(dir)
        self.processed_waypoint_pub.publish(ProcessedWaypoint(valid_waypoint[0], valid_waypoint[1], heading, order, True))

    def reached_waypoint_callback(self, data):
        self.order = data.order

    def pose_callback(self, data):
        self.pos[0] = data.pose.position.x
        self.pos[1] = data.pose.position.y
        self.heading = euler_from_quaternion(data.pose.orientation)[2]

    def occupancy_callback(self, data):
        self.occupancy_width = data.info.width
        self.occupancy_height = data.info.height
        self.occupancy_resolution = data.info.resolution
        self.occupancy = np.array(data.data).reshape((self.occupancy_rows, self.occupancy_width))

if __name__ == '__main__':
    local_planner = LocalPlanner()
    local_planner.run()