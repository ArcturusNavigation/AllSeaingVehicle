#!/usr/bin/env python
from collections import namedtuple
from six.moves import xrange

import rospy
import tf
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import numpy as np 

from std_msgs.msg import String 
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose, PoseWithCovarianceStamped, Twist
import message_filters

from pilot_suite.msg import ProcessedWaypoint, WaypointReached, SkipWaypoint, VelocityCommand, ProcessedTask
from pilot_suite.srv import GoToWaypoint, GoToWaypointResponse

from pilot_suite.geom_utils import quaternion_from_angle

from geometry_msgs.msg import Quaternion
from sklearn.decomposition import PCA
import tf

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


class Waypoint():
    def __init__(self,x,y,z,heading):
        self.x = x 
        self.y = y 
        self.z = z 
        self.heading = heading

class Task():
    def __init__(self,task_topic, task_timeout=60, nav_control= False, pause_queue=False):
        self.task_topic = task_topic
        self.task_timeout = task_timeout
        self.nav_control = nav_control
        self.pause_queue = pause_queue

class PilotModeShifter():
    """
    This class is responsible for commanding the vehicle using a controller running the Ardupilot autopilot.
    While running, it can be in one of many modes:
    - IDLE: waiting for a waypoint to be received, low power mode
    - TASK: executing a task, with one of the following submodes
        - TASK_NAV: Navigation demonstration and avoid the crowd task
        - TASK_TURN: Turn task 
        - TASK_DOCK: Docking task
    - TRAVEL: traveling to a waypoint
    - HOLD: holding at a waypoint
    """
    def __init__(self, queue=[]):
        self.state = State()
        self.imu = Imu()
        self.local_position = PoseStamped()
        self.init_local_position = None
        self.set_arming_srv = None
        self.set_mode_srv = None
        
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

        rospy.Subscriber('mavros/state', State, self.state_callback)
        rospy.Subscriber('mavros/imu/data', Imu, self.imu_callback)
        

        self.sub_topics_ready = {
            key: False
            for key in ['state', 'imu', 'local_pos', 'global_pos']
        }

        self.set_velocity = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=5)
        self.toggle_task = rospy.Publisher('pilot_suite/task', String, queue_size= 5)
        self.send_pose = rospy.Publisher('pilot_suite/pose', PoseStamped, queue_size=5)
        self.position_transform_broadcaster = tf.TransformBroadcaster()
        self.task_manager = dict()

        self.wait_for_topics(60)
        self.set_mode("Loiter", 5)
        self.set_arm(True, 5)


    def wait_for_topics(self, timeout):
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1 # Hz
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                rospy.loginfo("subscribed topics ready in {0} seconds".format(i / loop_freq))
                break
            elif i > 20:
                rospy.loginfo("Not all topics are ready but continuing anyway")
                break
            unready_topics = []
            for topic in self.sub_topics_ready:
                if not self.sub_topics_ready[topic]:
                    unready_topics.append(topic)
            rospy.loginfo("waiting for topics: {0}".format(unready_topics))
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo(f"armed state changed from {self.state.armed} to {data.armed}")

        if self.state.connected != data.connected:
            rospy.loginfo(f"connected changed from {self.state.connected} to {data.connected}")

        if self.state.mode != data.mode:
            rospy.loginfo(f"mode changed from {self.state.mode} to {data.mode}")

        if self.state.system_status != data.system_status:
            rospy.loginfo(f"system_status changed from {MAV_STATE_ENUM[self.state.system_status]} to {MAV_STATE_ENUM[data.system_status]}")

        self.state = data

        # mavros publishes a disconnected state message on init
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    def imu_callback(self, data):
        self.imu = data
        if not self.sub_topics_ready['imu']:
            self.sub_topics_ready['imu'] = True


    def set_arm(self, arm, timeout):
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        loop_freq = 1 # Hz
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                rospy.loginfo(f"set arm success in {i/loop_freq} seconds")
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
            except rospy.ROSException as e:
                rospy.logerr(e)
    
    def set_mode(self, mode, timeout):
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        loop_freq = 1 # Hz
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                rospy.loginfo(f"set mode success in {i/loop_freq} seconds")
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)
        

if __name__ == '__main__':
    try:
        rospy.init_node('pilot_shifter')
        pilotShifter = PilotModeShifter()
        pilotShifter.run()
    except rospy.ROSInterruptException:
        pass
