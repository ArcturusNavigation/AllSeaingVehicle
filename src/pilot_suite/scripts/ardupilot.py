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
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose, PoseWithCovarianceStamped, Twist, GeoPoint
import message_filters

from pilot_suite.msg import ProcessedWaypoint, WaypointReached, SkipWaypoint, VelocityCommand, ProcessedTask
from pilot_suite.srv import GoToWaypoint, GoToWaypointResponse
from sensor_suite.msg import LabeledBoundingBox2DArray, LabeledBoundingBox2D

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

ManagedTask = namedtuple('MangaedTask', ['start', 'timeout'])


buoy_task = ProcessedTask()
buoy_task.task_topic = "navigation_pilot"
buoy_task.task_timeout = 60
EXAMPLE_QUEUE = [buoy_task]

class Ardupilot():
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
        self.global_position = GeoPoseStamped()
        self.init_local_position = None
	self.init_global_position = None
        self.set_arming_srv = None
        self.set_mode_srv = None

        self.queue = queue
        self.skipped_waypoints = []
        self.temp_waypoint = None
        self.temp_twist = None
        self.current_order = 0
        self.task_manager = dict()

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
        # rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_position_callback)
        pix_pos_sub = message_filters.Subscriber('mavros/local_position/pose', PoseStamped)
	pix_pos_gps_sub = message_filters.Subscriber("mavros/global_position/pose", GeoPoseStamped)
        zed_pos_sub = message_filters.Subscriber('/zed2i/zed_node/pos', PoseStamped) #TODO: Get actual topic
        ts = message_filters.ApproximateTimeSynchronizer([pix_pos_sub, zed_pos_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.combined_pos_callback)

        self.sub_topics_ready = {
            key: False
            for key in ['state', 'imu', 'local_pos', 'global_pos']
        }

        rospy.Subscriber('pilot_suite/processed_waypoint', ProcessedWaypoint, self.waypoint_callback)
        rospy.Subscriber('pilot_suite/waypoint_skip', SkipWaypoint, self.waypoint_skip_callback)
        rospy.Subscriber('pilot_suite/velocity_command', VelocityCommand, self.velocity_command_callback)
        rospy.Subscriber('pilot_suite/processed_task', ProcessedTask, self.task_callback)
        rospy.Subscriber('pilot_suite/task_status', String, self.task_status_callback)


        self.send_waypoint_reached = rospy.Publisher('pilot_suite/waypoint_reached', WaypointReached, queue_size=5)

        self.set_local_setpoint = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=5)
	self.set_global_setpoint = rospy.Publisher('mavros/setpoint_position/global', GlobalPoseStamped, queue_size=5)

        self.set_velocity = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=5)
        self.toggle_task = rospy.Publisher('pilot_suite/task', String, queue_size= 5)
        self.send_pose = rospy.Publisher('pilot_suite/pose', PoseStamped, queue_size=5)
        self.position_transform_broadcaster = tf.TransformBroadcaster()
        self.task_manager = dict()

        if USE_FAKE_GPS_FROM_ZED:
            rospy.Subscriber('zed/zed_node/pose_with_covariance', PoseWithCovarianceStamped, self.zed_pose_callback)
            self.send_fake_gps = rospy.Publisher('mavros/mocap/pose_cov', PoseWithCovarianceStamped, queue_size=5)

        rospy.Subscriber('/clicked_point', Point, self.clicked_point_callback)

        self.wait_for_topics(60)
        self.set_mode("Guided", 5)
        self.set_arm(True, 5)

        rate = rospy.Rate(30) # frequency in Hz at which we can update next waypoint
        while not rospy.is_shutdown():
            # constantly update our current setpoint
            self.update_tasks()
            self.update_controller()
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

    def clicked_point_callback(self, data):
        q = [self.local_position.pose.orientation.x, self.local_position.pose.orientation.y, self.local_position.pose.orientation.z, self.local_position.pose.orientation.w]
        self.queue = [[(data.point.x, data.point.y, tf.transformations.euler_from_quaternion(q)[2])]]
        self.current_order = 0
        self.temp_waypoint = None

    def go_to_waypoint(self, req):
        self.queue = [[(self.local_position.pose.position.x + req.x, 
            self.local_position.pose.position.y + req.y, req.heading)]]
        self.current_order = 0
        self.temp_waypoint = None
        return GoToWaypointResponse()

    def get_curr_waypoint(self):
        while self.current_order in self.skipped_waypoints:
            # make sure the local planner is up to date with which waypoint is next
            self.waypoint_reached_pub.publish(WaypointReached(self.current_order))
            self.current_order += 1
        if self.temp_waypoint is not None:
            return self.temp_waypoint
        if len(self.queue) <= self.current_order:
            return None
        return self.queue[self.current_order]

    # subscribes to the local position of the boat and updates the next
    # waypoint if the local position is within acceptance radius of current setpoint
    def local_position_callback(self, data):
        if self.init_local_position is None:
            self.init_local_position = data
        self.local_position = data

        # rospy.loginfo(f"local position: {(data.pose.position.x, data.pose.position.y, data.pose.position.z)}")

        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

        # publish position to pilot_suite/position
        self.send_pose.publish(data)
        curr_waypoint = self.get_curr_waypoint()
        
        if curr_waypoint is None or isinstance(curr_waypoint, ProcessedTask):
            return

        dist_sq = (self.local_position.pose.position.x - curr_waypoint[0]) ** 2 + (self.local_position.pose.position.y - curr_waypoint[1]) ** 2
        if dist_sq < ACCEPTANCE_RADIUS ** 2:
            if self.temp_waypoint != None:
                self.temp_waypoint = None
            else:
                self.send_waypoint_reached.publish(WaypointReached(self.current_order))
                self.current_order += 1
            self.update_controller()
    
    # combines pixhawk and zed camera data
    def combined_pos_callback(self, pixhawk_data, zed_data):
        self.combined_pos = PoseStamped()
        self.combined_pos.header = pixhawk_data.header
        self.combined_pos.pose.position.x = (pixhawk_data.pose.position.x + zed_data.pose.position.x) / 2
        self.combined_pos.pose.position.y = (pixhawk_data.pose.position.y + zed_data.pose.position.y) / 2
        self.combined_pos.pose.position.z = (pixhawk_data.pose.position.z + zed_data.pose.position.z) / 2
        self.combined_pos.pose.orientation = pixhawk_data.pose.orientation

        if not self.sub_topics_ready['combined_pos']:
            self.sub_topics_ready['combined_pos'] = True

    # subscribes to incoming waypoints and adds them to a list in
    # the correct order to be processed
    def waypoint_callback(self, waypoint):
        if waypoint.is_temp:
            self.temp_waypoint = Waypoint(waypoint.x, waypoint.y, waypoint.heading)
            return
            
        while len(self.queue) <= waypoint.order:
            self.queue.append(())
        self.queue[waypoint.order] = Waypoint(waypoint.x, waypoint.y, waypoint.heading)

    def waypoint_skip_callback(self, waypoint_skip):
        order = waypoint_skip.order
        self.skipped_waypoints.append(order)

    # TODO: Finish
    def task_callback(self, task):
        while len(self.queue) <= task.order:
            self.queue.append(())
        self.queue[task.order] = Task(task.task_topic, task.task_timeout, task.nav_control)

    def task_status_callback(self, msg):
        task = msg.data
        if task not in self.task_manager:
            rospy.loginfo(f"Untracked task({task}) has completed. Shutting off task")
        self.toggle_task("-" + task)# Just in case task did not shut itself off
            
    def velocity_command_callback(self, velocity_command):
        if velocity_command.cancel:
            self.temp_twist = None
        else:
            self.temp_twist = velocity_command.twist
    
    # TODO: Write safety controller and connect topics
    def safety_command_callback(self, safety_command):
        raise NotImplementedError("Safety command callback not implemented")

    def zed_pose_callback(self, pose_with_cov):
        self.send_fake_gps.publish(pose_with_cov)

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
        
    def update_tasks(self):
        for task in self.task_manager:
            if (rospy.Time.now() - self.task_manager[task].start).to_sec() > self.task_manager[task].timeout:
                self.toggle_task.publish(String("-" + task))
                del self.task_manager[task]
                if self.temp_twist is not None: # In case the task was a velocity controller 
                    self.temp_twist = None
                rospy.loginfo(f"Task {task} timed out")
            
    # takes the current setpoint and sends it to the FCU
    # needs to be called at minimum 2 Hz
    def update_controller(self):
        if self.temp_twist != None:
            self.set_velocity.publish(self.temp_twist)
            rospy.loginfo(f"setting velocity: {self.temp_twist.linear.x} {self.temp_twist.angular.z}")
            return

        waypoint = self.get_curr_waypoint()
        if waypoint is None:
            # rospy.loginfo("no waypoint set")
            self.set_local_setpoint.publish(self.local_position)
        elif isinstance(waypoint, ProcessedTask):
            task = waypoint.task_topic
            if task in self.task_manager:
                rospy.loginfo(f"Task {task} already running")
            else:
                self.task_manager[task] = ManagedTask(rospy.Time.now(), waypoint.task_timeout)
                self.toggle_task.publish(String("+" + task))
            self.current_order += 1
            return 
        else:
            waypoint_x, waypoint_y, waypoint_heading = waypoint[0], waypoint[1], waypoint[2]
            rospy.loginfo(f"setting setpoint: {waypoint_x} {waypoint_y} {waypoint_heading}")
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
        pilot = Ardupilot(queue=EXAMPLE_QUEUE)
        pilot.run()
    except rospy.ROSInterruptException:
        pass
