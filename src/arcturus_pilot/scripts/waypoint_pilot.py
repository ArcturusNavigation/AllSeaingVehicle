#!/usr/bin/env python

import rospy
import tf
from mavros_msgs.msg import State, Waypoint
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose, PoseWithCovarianceStamped

from arcturus_pilot.msg import Waypoint
from arcturus_pilot.srv import GoToWaypoint, GoToWaypointResponse

from six.moves import xrange

ACCEPTANCE_RADIUS = 0.2
USE_FAKE_GPS_FROM_ZED = True

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
        self.global_position = NavSatFix()
        self.local_position = PoseStamped()
        self.init_local_position = None
        self.set_arming_srv = None
        self.set_mode_srv = None

        self.waypoints = []
        self.current_order = 0

    def run(self):
        rospy.init_node('pilot', anonymous=True)

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

        rospy.Service('go_to_waypoint', GoToWaypoint, go_to_waypoint)
        
        rospy.Subscriber('mavros/state', State, self.state_callback)
        rospy.Subscriber('mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('mavros/global_position/global', NavSatFix, self.global_position_callback)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self.sub_topics_ready = {
            key: False
            for key in ['state', 'imu', 'global_pos', 'local_pos']
        }

        rospy.Subscriber('waypoint', Waypoint, self.waypoint_callback)

        self.set_local_setpoint = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=5)

        if USE_FAKE_GPS_FROM_ZED:
            rospy.Subscriber('zed/zed_node/pose_with_covariance', PoseWithCovarianceStamped, self.zed_pose_callback)
            self.send_fake_gps = rospy.Publisher('mavros/mocap/pose_cov', PoseWithCovarianceStamped, queue_size=5)

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

    def global_position_callback(self, data):
        self.global_position = data
        if not self.sub_topics_ready['global_position']:
            self.sub_topics_ready['global_position'] = True

    def go_to_waypoint(self, req):
        self.waypoints = [(self.local_position.pose.position.x + req.x, 
            self.local_position.pose.position.y + req.y, req.heading)]
        return GoToWaypointResponse()

    # subscribes to the local position of the boat and updates the next
    # waypoint if the local position is within acceptance radius of current setpoint
    def local_position_callback(self, data):
        if self.init_local_position == None:
            self.init_local_position = data
        self.local_position = data

        rospy.loginfo("local position: {0, 1, 2}}".format(data.pose.position.x, data.pose.position.y, data.pose.position.z))

        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True
        
        if len(self.waypoints == 0):
            return
        
        curr_setpoint = self.waypoints[0]
        dist_sq = (self.local_position.pose.x - curr_setpoint[0]) ** 2 + (self.local_position.pose.y - curr_setpoint[1]) ** 2
        if dist_sq < ACCEPTANCE_RADIUS ** 2:
            self.waypoints.pop(0)
            self.current_order += 1
            self.send_setpoint()

    # subscribes to incoming waypoints and adds them to a list in
    # the correct order to be processed
    def waypoint_callback(self, waypoint):
        adjusted_order = waypoint.order - self.current_order
        if adjusted_order < 0:
            return
        while len(self.waypoints < adjusted_order):
            self.waypoints.append(None)
        self.waypoints[adjusted_order] = (waypoint.x, waypoint.y, waypoint.heading)

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
        waypoint = None
        if len(self.waypoints) == 0:
            self.set_local_setpoint.publish(self.local_position)
        else:
            waypoint = self.waypoints[0]
            waypoint_x, waypoint_y, waypoint_heading = waypoint[0], waypoint[1], waypoint[2]
            rospy.loginfo("setting setpoint: {0, 1, 2}".format(waypoint_x, waypoint_y, waypoint_heading))

            pose = PoseStamped()
            pose.header.stamp = rospy.now()
            pose.header.frame_id = '/local_origin'
            pose.pose = Pose()
            pose.pose.position = Point(x = waypoint_x + self.init_local_position.pose.position.x, 
                y = waypoint_y + self.init_local_position.pose.position.y, z = 0)
            pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, waypoint_heading))
            self.set_local_setpoint.publish(pose)

if __name__ == '__main__':
    try:
        pilot = WaypointPilot()
        pilot.run()
    except rospy.ROSInterruptException:
        pass
