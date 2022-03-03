#!/usr/bin/env python

import rospy
from mavros_msgs.msg import State, WaypointReached, Waypoint, WaypointList
from mavros_msgs.srv import CommandBool, SetMode, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil

from arcturus_pilot.msg import Waypoint

from six.moves import xrange

class WaypointPilot():
    def __init__(self):
        self.state = State()
        self.imu = Imu()
        self.global_position = NavSatFix()
        self.local_position = PoseStamped()
        self.set_arming_srv = None
        self.set_mode_srv = None

        self.waypoints = []
        self.current_order = 0
        self.mission_item_reached = -1

    def run(self):
        rospy.init_node('pilot', anonymous=True)

        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            rospy.logerr('failed to connect to services')
            return
        
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.push_waypoints_srv = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
        
        rospy.Subscriber('mavros/state', State, self.state_callback)
        rospy.Subscriber('mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('mavros/global_position/global', NavSatFix, self.global_position_callback)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self.sub_topics_ready = {
            key: False
            for key in ['state', 'imu', 'global_pos', 'local_pos']
        }

        rospy.Subscriber('mavros/mission/reached', WaypointReached, self.mission_item_reached_callback)

        rospy.Subscriber('waypoint', Waypoint, self.waypoint_callback)

        self.wait_for_topics(60)
        self.set_mode("Auto", 5)
        self.set_arm(True, 5)

        rate = rospy.Rate(10) # frequency in Hz at which we can update next waypoint
        while not rospy.is_shutdown():
            # send everything in queue to the controller
            if len(self.waypoints != 0):
                self.push_waypoints(self.waypoints, 10)
                self.waypoints = []
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
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name))

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

    def local_position_callback(self, data):
        self.local_position = data
        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def waypoint_callback(self, waypoint):
        adjusted_order = waypoint.order - self.current_order
        if adjusted_order < 0:
            return
        while len(self.waypoints < adjusted_order):
            self.waypoints.append(None)
        self.waypoints[adjusted_order] = self.create_waypoint(waypoint.x, waypoint.y)

    def create_waypoint(waypoint_x, waypoint_y):
        """
        Frame values:
            uint8 FRAME_GLOBAL = 0
            uint8 FRAME_LOCAL_NED = 1
            uint8 FRAME_MISSION = 2
            uint8 FRAME_GLOBAL_REL_ALT = 3
            uint8 FRAME_LOCAL_ENU = 4

        Command values:
            uint16 NAV_WAYPOINT = 16
            uint16 NAV_LOITER_UNLIM = 17
            uint16 NAV_LOITER_TURNS = 18
            uint16 NAV_LOITER_TIME = 19
            uint16 NAV_RETURN_TO_LAUNCH = 20
            uint16 NAV_LAND = 21
            uint16 NAV_TAKEOFF = 22

        https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
        Params for waypoint:
            1: hold time in seconds
            2: accept radius in m
            3: pass radius in m
            4: yaw n deg (use NaN to use current system)
        """
        return Waypoint(frame=1, command=16, is_current=False, autocontinue=True, param1=0.0, param2=0.25,
            param3=0, param4=float('nan'), x_lat=waypoint_x, y_lon=waypoint_y, z_alt=0)

    def mission_item_reached_callback(self, data):
        if self.mission_item_reached != data.wp_seq:
            rospy.loginfo("mission item reached: {0}".format(data.wp_seq))
            self.mission_item_reached = data.wp_seq

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

    def push_waypoints(self, waypoints, timeout):
        rospy.loginfo("pushing waypoints")
        loop_freq = 1 # Hz
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            try:
                res = self.push_waypoints_srv(self.current_order, waypoints)
                if res.success:
                    self.current_order += len(waypoints)
                    break
                else:
                    rospy.logerr("failed to send waypoints")
            except rospy.ServiceException as e:
                rospy.logerr(e)
        
            try:
                rate.sleep()
            except rospy.RosException as e:
                rospy.logerr(e)

if __name__ == '__main__':
    try:
        pilot = WaypointPilot()
        pilot.run()
    except rospy.ROSInterruptException:
        pass
