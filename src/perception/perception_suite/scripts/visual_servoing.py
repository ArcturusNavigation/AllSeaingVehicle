#! /usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

disToObstacle = 1


def callback(msg):
    #where msg is a list of tuple coordinates
    #say max is 300 degrees per second
    rospy.loginfo(msg.data)  # prints on terminal
    max_x = 200
    max_y = 200

    top_left_b1 = msg.data[0]
    bottom_right_b1 = msg.data[1]

    top_left_b2 = msg.data[2]
    bottom_right_b2 = msg.data[3]

    center_x_b1 = (top_left_b1[0] + bottom_right_b1[0])/2
    center_y_b1 = (top_left_b1[1] + bottom_right_b1[1])/2

    center_x_b2 = (top_left_b2[0] + bottom_right_b2[0])/2
    center_y_b2 = (top_left_b2[1] + bottom_right_b2[1])/2

    center_both_x = (center_x_b1 + center_x_b2)/2
    center_both_y = (center_y_b1 + center_y_b2)/2

    center_frame = (max_x/2, max_y/2)
 

    #distances are negative when to the right of the center frame
    #positive when to the left
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

#want maximum to be 300 degrees(5.24 radians) per second if we are at
#max distance(100 pixels)
#want it to decrease linearly until we are some small distance away
#when only 10 away, bring the velocity down to 0
#negative turn rate corresponds to right turn
#positive corresponds to left turn

if abs(dist_x) > 10:
    #will be negative(turn right) when dist_x is positive(too far left)
    turn_speed = math.radians(dist_x * -3 / 2)
else:
    turn_speed = 0

move.angular.x = turn_speed


rospy.spin()
