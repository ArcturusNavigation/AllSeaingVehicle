import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point
import tf

def angle_from_dir(dir):
    return np.arctan2(dir[1], dir[0])

def quaternion_from_dir(dir):
    angle = np.arctan2(dir[1], dir[0])

    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

def create_pose_stamped(pos, dir):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'map'
    pose.pose.position = Point(x = pos[0], y = pos[1], z = 0)
    pose.pose.orientation = quaternion_from_dir(dir)

    return pose
