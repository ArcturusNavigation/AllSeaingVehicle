#!/usr/bin/env python

"""
Publishes sensor transforms 
"""
import rospy 
from geometry_msgs.msg import Transform 
import tf
GPs -> Lidar: (x:.0254, y:.8763, z:.127)
# Zed->Lidar: (x:0, y: .1143, z:.1905)
SENSORS = {'camera': {'frame_id': 'base_link', 
'position': [0, -0.01, 2], 
'orientation':  tf.transformations.quaternion_from_euler(-.0254, .1143+.8763, .1905 - .127)
}, 'velodyne': {'frame_id': 'base_link', 
'position': [0, 0, 0.095455], 
'orientation': tf.transformations.quaternion_from_euler(-.0254, .8763, .1905)}}
def broadcast_sensor_poses():
    br = tf.TransformBroadcaster()
    for sensor in SENSORS:
        br.sendTransform(SENSORS[sensor]['position'], SENSORS[sensor]['orientation'], rospy.Time.now(), sensor, SENSORS[sensor]['frame_id'])

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        broadcast_sensor_poses()
        rate.sleep()
    


