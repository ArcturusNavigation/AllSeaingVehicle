#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('broadcast_map_transform')

    broadcaster = tf2_ros.TransformBroadcaster()
    transform = geometry_msgs.msg.TransformStamped()

    transform.header.frame_id = "map"
    transform.child_frame_id = "base_link"

    transform.transform.translation.x = 35.000
    transform.transform.translation.y = 20.000
    transform.transform.translation.z = 0.000

    quat = tf.transformations.quaternion_from_euler(float(0), float(0), float(0))
    transform.transform.rotation.x = quat[0]
    transform.transform.rotation.y = quat[1]
    transform.transform.rotation.z = quat[2]
    transform.transform.rotation.w = quat[3]



    pub = rospy.Publisher('/planner/goal', geometry_msgs.msg.PoseStamped, queue_size=10)
    goal = geometry_msgs.msg.PoseStamped()
    goal.header.frame_id = "map"

    goal.pose.position = geometry_msgs.msg.Point(x = 35.00, y = 35.000, z = 0.000)
    goal.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 3.14))

    def clicked_point_callback(data):
        rospy.loginfo('sending new goal')
        goal.pose.position = data.point 

    sub = rospy.Subscriber('/clicked_point', geometry_msgs.msg.PointStamped, clicked_point_callback)


    rate = rospy.Rate(10) # frequency in Hz at which we can update next waypoint
    while not rospy.is_shutdown():
        transform.header.stamp = rospy.Time.now()
        goal.header.stamp = rospy.Time.now()
        broadcaster.sendTransform(transform)
        pub.publish(goal)
        rate.sleep()

