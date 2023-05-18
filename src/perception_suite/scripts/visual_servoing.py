#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

disToObstacle = 1


def callback(msg):
    rospy.loginfo(msg.data)  # prints on terminal
    max_x = 200
    max_y = 200

    top_left_b1 = msg.data[0]
    bottom_right_b1 = msg.data[1]

    top_left_b2 = msg.data[2]
    bottom_right_b2 = msg.data[3]

    center_x_b1 = (top_left_b1[0] + bottom_right_b1[0])/2
    center_y_b1 = (top_left_b1[1] + bottom_right_b1[1])/2

    center_x_b2 = (top_left_b2[0] + bottom_right_b1[0])/2
    center_y_b2 = (top_left_b2[1] + bottom_right_b2[1])/2

    center_both_x = (center_x_b1 + center_x_b2)/2
    center_both_y = (center_y_b1 + center_y_b2)/2

    center_frame = (max_x/2, max_y/2)
    dist_x = center_frame[0] - center_both_x
    dist_y = center_frame[1] - center_both_y


    # Controlller takes in the information from the boudning boxes from msg that it subscribes to

    ################# Implement Controller from Bounding Box to Velocity Here #################

    ##############################################################
    # Some PWM Command
    pwm = 100
    pub.publish(move)



rospy.init_node("visual_servoing_node")
sub = rospy.Subscriber(
    "/perception/bounding_box_stuff", Twist, callback
)  # We subscribe to the boats velocity
pub = rospy.Publisher("/visual_servoing/velocity", Twist, queue_size=2)
rate = rospy.Rate(2)
move = Twist()


move.angular.x = dist_x


rospy.spin()
