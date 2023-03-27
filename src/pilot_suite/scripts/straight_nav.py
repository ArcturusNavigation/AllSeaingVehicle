#!/usr/bin/env python3
import numpy as np
import cv2
import os
import statistics
import rospy

from geometry_msgs.msg import Vector3, Twist

from task_node import TaskNode

class StraightNavNode(TaskNode):

    def __init__(self, const_vel):

        super().__init__('straight_nav_task')

        self.velocity_pub = rospy.Publisher(
            '/pilot_suite/straight_nav/velocity_pub', Twist, queue_size=1)
        
        self.VELOCITY = const_vel

   
    def run(self):
        
        while not rospy.is_shutdown():

            linear_velocity, angular_velocity = Vector3(), Vector3()

            linear_velocity.x = self.VELOCITY
            linear_velocity.y = 0
            linear_velocity.z = 0

            angular_velocity.x = 0
            angular_velocity.y = 0
            angular_velocity.z = 0

            velocity_msg = Twist()

            velocity_msg.linear = linear_velocity
            velocity_msg.angular = angular_velocity

            self.velocity_pub.publish(velocity_msg)

            self.rate.sleep()


if __name__ == '__main__':

    print("Python Script Running!")
    rospy.init_node('straight_nav_task')
    task_node = StraightNavNode(10.0)
    task_node.active = True
    task_node.run()
