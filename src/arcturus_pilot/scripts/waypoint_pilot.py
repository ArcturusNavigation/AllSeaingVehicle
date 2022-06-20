#!/usr/bin/env python

import rospy
import tf
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose, PoseWithCovarianceStamped, Twist

from arcturus_pilot.msg import ProcessedWaypoint, WaypointReached, SkipWaypoint, VelocityCommand
from arcturus_pilot.srv import GoToWaypoint, GoToWaypointResponse

from geom_helper import quaternion_from_angle

from six.moves import xrange

ACCEPTANCE_RADIUS = 0.2
USE_FAKE_GPS_FROM_ZED = False

# see https://mavlink.io/en/messages/common.html#MAV_STATE
MAV_STATE_ENUM = {
    0: 'UNINIT',
    1: 'BOOT',
    2: 'CALIBRATING',
    3: 'STANDBY',
    4: 'ACTIVE',
    5: 'CRITICAL',
    6: 'EMERGENCY',
    7: 'POWEROFF',
    8: 'FLIGHT_TERMINATION'
}

class WaypointPilot():
    def __init__(self):
        self.state = State()
        self.imu = Imu()
        self.local_position = PoseStamped()
        self.init_local_position = None
        self.set_arming_srv = None
        self.set_mode_srv = None

        self.waypoints = []
        self.skipped_waypoints = []
        self.temp_waypoint = None
        self.temp_twist = None
        self.current_order = 0

    def run(self):
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            rospy.logerr('failed to connect to services')
            return
        
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)

        rospy.Service('go_to_waypoint', GoToWaypoint, self.go_to_waypoint)
        
        rospy.Subscriber('mavros/state', State, self.state_callback)
        rospy.Subscriber('mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self.sub_topics_ready = {
            key: False
            for key in ['state', 'imu', 'local_pos']
        }

        rospy.Subscriber('arcturus_pilot/processed_waypoint', ProcessedWaypoint, self.waypoint_callback)
        rospy.Subscriber('arcturus_pilot/waypoint_skip', SkipWaypoint, self.waypoint_skip_callback)
        rospy.Subscriber('arcturus_pilot/velocity_command', VelocityCommand, self.velocity_command_callback)
        self.send_waypoint_reached = rospy.Publisher('arcturus_pilot/waypoint_reached', WaypointReached, queue_size=5)
        self.set_local_setpoint = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=5)
        self.set_velocity = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=5)
        self.send_pose = rospy.Publisher('arcturus_pilot/pose', PoseStamped, queue_size=5)
        self.position_transform_broadcaster = tf.TransformBroadcaster()

        if USE_FAKE_GPS_FROM_ZED:
            rospy.Subscriber('zed/zed_node/pose_with_covariance', PoseWithCovarianceStamped, self.zed_pose_callback)
            self.send_fake_gps = rospy.Publisher('mavros/mocap/pose_cov', PoseWithCovarianceStamped, queue_size=5)

        rospy.Subscriber('/clicked_point', Point, self.clicked_point_callback)

        self.wait_for_topics(60)
        self.set_mode("Guided", 5)
        self.set_arm(True, 5)

        rate = rospy.Rate(10) # frequency in Hz at which we can update next waypoint
        while not rospy.is_shutdown():
            # constantly update our current setpoint
            self.send_setpoint()
            rate.sleep()

        self.set_arm(False, 5)

    def wait_for_topics(self, timeout):
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1 # Hz
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                rospy.loginfo("subscribed topics ready in {0} seconds".format(i / loop_freq))
                break
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, data.mode))

        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                MAV_STATE_ENUM[self.state.system_status],
                MAV_STATE_ENUM[data.system_status]))

        self.state = data

        # mavros publishes a disconnected state message on init
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    def imu_callback(self, data):
        self.imu = data
        if not self.sub_topics_ready['imu']:
            self.sub_topics_ready['imu'] = True

    def clicked_point_callback(self, data):
        q = [self.local_position.pose.orientation.x, self.local_position.pose.orientation.y, self.local_position.pose.orientation.z, self.local_position.pose.orientation.w]
        self.waypoints = [[(data.point.x, data.point.y, tf.transformations.euler_from_quaternion(q)[2])]]
        self.current_order = 0
        self.temp_waypoint = None

    def go_to_waypoint(self, req):
        self.waypoints = [[(self.local_position.pose.position.x + req.x, 
            self.local_position.pose.position.y + req.y, req.heading)]]
        self.current_order = 0
        self.temp_waypoint = None
        return GoToWaypointResponse()

    def get_curr_waypoint(self):
        while self.current_order in self.skipped_waypoints:
            self.current_order += 1

        if self.temp_waypoint is not None:
            return self.temp_waypoint
        if len(self.waypoints) <= self.current_order:
            return None
        return self.waypoints[self.current_order]

    # subscribes to the local position of the boat and updates the next
    # waypoint if the local position is within acceptance radius of current setpoint
    def local_position_callback(self, data):
        if self.init_local_position == None:
            self.init_local_position = data
        self.local_position = data

        rospy.loginfo("local position: {0, 1, 2}}".format(data.pose.position.x, data.pose.position.y, data.pose.position.z))

        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

        # publish position to arcturus_pilot/position
        self.send_pose.publish(data)
        
        curr_waypoint = self.get_curr_waypoint()
        if curr_waypoint is None:
            return
        
        dist_sq = (self.local_position.pose.x - curr_waypoint[0]) ** 2 + (self.local_position.pose.y - curr_waypoint[1]) ** 2
        if dist_sq < ACCEPTANCE_RADIUS ** 2:
            if self.temp_waypoint != None:
                self.temp_waypoint = None
            else:
                self.send_waypoint_reached.publish(WaypointReached(self.current_order))
                self.current_order += 1
            self.send_setpoint()

    # subscribes to incoming waypoints and adds them to a list in
    # the correct order to be processed
    def waypoint_callback(self, waypoint):
        if waypoint.is_temp:
            self.temp_waypoint = (waypoint.x, waypoint.y, waypoint.heading)
            return
            
        while len(self.waypoints) <= waypoint.order:
            self.waypoints.append(())
        self.waypoints[waypoint.order] = (waypoint.x, waypoint.y, waypoint.heading)

    def waypoint_skip_callback(self, waypoint_skip):
        order = waypoint_skip.order
        self.skipped_waypoints.append(order)

    def velocity_command_callback(self, velocity_command):
        if velocity_command.cancel:
            self.temp_twist = None
        else:
            self.temp_twist = velocity_command.twist

    def zed_pose_callback(self, pose_with_cov):
        self.send_fake_gps.publish(pose_with_cov)

    def set_arm(self, arm, timeout):
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        loop_freq = 1 # Hz
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                rospy.loginfo("set arm success in {0} seconds".format(i / loop_freq))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            
            try:
                rate.sleep()
            except rospy.RosException as e:
                rospy.logerr(e)
    
    def set_mode(self, mode, timeout):
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        loop_freq = 1 # Hz
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                rospy.loginfo("set mode success in {0} seconds".format(i / loop_freq))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)
                    if not res.success:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            
            try:
                rate.sleep()
            except rospy.RosException as e:
                rospy.logerr(e)
    
    # takes the current setpoint and sends it to the FCU
    # needs to be called at minimum 2 Hz
    def send_setpoint(self):
        if self.temp_twist != None:
            self.set_velocity.publish(self.temp_twist)
            return

        waypoint = self.get_curr_waypoint()
        if waypoint == None:
            rospy.logerr("no waypoint set")
            self.set_local_setpoint.publish(self.local_position)
        else:
            waypoint_x, waypoint_y, waypoint_heading = waypoint[0], waypoint[1], waypoint[2]
            rospy.loginfo("setting setpoint: " + str(waypoint_x) + " " + str(waypoint_y) + " " + str(waypoint_heading))

            pose = PoseStamped()
            pose.header.stamp = rospy.now()
            pose.header.frame_id = '/local_origin'
            pose.pose = Pose()
            pose.pose.position = Point(x = waypoint_x, y = waypoint_y, z = 0)
            pose.pose.orientation = quaternion_from_angle(waypoint_heading)
            self.set_local_setpoint.publish(pose)

if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_pilot')
        pilot = WaypointPilot()
        pilot.run()
    except rospy.ROSInterruptException:
        pass
