import rospy
from arcturus_pilot.msg import RawWaypoint, ProcessedWaypoint, WaypointReached
from geometry_msgs.msg import PoseStamped, OccupancyGrid
from tf import euler_from_quaternion

class LocalPlanner():
    def __init__(self):
        self.raw_waypoint_sub = rospy.Subscriber('arcturus_pilot/raw_waypoint', RawWaypoint, self.raw_waypoint_callback)
        self.processed_waypoint_pub = rospy.Publisher('arcturus_pilot/processed_waypoint', ProcessedWaypoint)
        self.reached_waypoint_sub = rospy.Subscriber('arcturus_pilot/waypoint_reached', WaypointReached, self.reached_waypoint_callback)
        self.pose_sub = rospy.Subscriber('arcturus_pilot/pose', PoseStamped, self.pose_callback)
        self.occupancy_sub = rospy.Subscriber('sensor_suite/occupancy', OccupancyGrid, self.occupancy_callback)

        self.occupancy = None
        self.pos = [0, 0]
        self.heading = 0
        self.order = -1

    def run():
        rospy.init_node('local_planner')
        rospy.spin()

    def raw_waypoint_callback(self, data):
        (x, y, heading, order) = (data.x, data.y, data.heading, data.order)
        if self.order + 1 != order or self.occupancy is None:
            self.processed_waypoint_pub.publish(ProcessedWaypoint(x, y, heading, order, 0))
            return
        
        # TODO - implement drawing a line between current position and waypoint and seeing if it overlaps with 
        # anything in occupancy grid

    def reached_waypoint_callback(self, data):
        self.order = data.order

    def pose_callback(self, data):
        self.pos[0] = data.pose.position.x
        self.pos[1] = data.pose.position.y
        self.heading = euler_from_quaternion(data.pose.orientation)[2]

    def occupancy_callback(self, data):
        self.occupancy = data


if __name__ == '__main__':
    local_planner = LocalPlanner()
    local_planner.run()