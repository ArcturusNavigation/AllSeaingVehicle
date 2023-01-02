#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('broadcast_map_transform')

    cur_pos = ((10, 10, 0), tf.transformations.quaternion_from_euler(0, 0, 0))

    broadcaster = tf2_ros.TransformBroadcaster()
    transform = geometry_msgs.msg.TransformStamped()

    transform.transform.rotation.x = cur_pos[1][0]
    transform.transform.rotation.y = cur_pos[1][1]
    transform.transform.rotation.z = cur_pos[1][2]
    transform.transform.rotation.w = cur_pos[1][3]

    transform.header.frame_id = "map"
    transform.child_frame_id = "base_link"

    def clicked_point_callback(data):
        global cur_pos
        rospy.loginfo('sending new position')
        cur_pos = ((data.point.x, data.point.y, data.point.z), cur_pos[1])

    sub = rospy.Subscriber('/clicked_point', geometry_msgs.msg.PointStamped, clicked_point_callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation.x = cur_pos[0][0]
        transform.transform.translation.y = cur_pos[0][1]
        transform.transform.translation.z = cur_pos[0][2]

        broadcaster.sendTransform(transform)
        rate.sleep()

