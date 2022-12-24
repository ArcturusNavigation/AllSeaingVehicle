#!/usr/bin/env python

"""
Publishes sensor transforms 
"""
import rospy 
from geometry_msgs.msg import TransformStamped
import tf

import tf2_ros

SENSORS = {#'camera': {'frame_id': 'base_link', 
# 'position': [0, -0.01, 2], 
# 'orientation':  tf.transformations.quaternion_from_euler(-1.57, -1.57, 0)
# }, 'velodyne': {'frame_id': 'base_link', 
# 'position': [0, 0, 0.095455], 
# 'orientation': tf.transformations.quaternion_from_euler(0, -0, 0)},
'camera_to_velodyne_temp':{'frame_id': 'velodyne', # Placeholder until base_link frame issues are fixed
'position':[0, -.01, 2.0-.095455],
'orientation': tf.transformations.quaternion_from_euler(0, -0, 0)
}} 
def broadcast_sensor_poses():
    br = tf2_ros.StaticTransformBroadcaster()
    for sensor in SENSORS:
        static_transform = TransformStamped()
        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = SENSORS[sensor]['frame_id']
        static_transform.child_frame_id = sensor
        static_transform.transform.translation.x = SENSORS[sensor]['position'][0]
        static_transform.transform.translation.y = SENSORS[sensor]['position'][1]
        static_transform.transform.translation.z = SENSORS[sensor]['position'][2]
        static_transform.transform.rotation.x = SENSORS[sensor]['orientation'][0]
        static_transform.transform.rotation.y = SENSORS[sensor]['orientation'][1]
        static_transform.transform.rotation.z = SENSORS[sensor]['orientation'][2]
        static_transform.transform.rotation.w = SENSORS[sensor]['orientation'][3]
        br.sendTransform(static_transform)

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    broadcast_sensor_poses()
    print("Broadcasting sensor poses")
    rospy.spin()
    


