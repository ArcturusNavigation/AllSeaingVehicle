#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist

disToObstacle = 1

feedback_linear_velocity = 0
feedback_angular = 0

commanded_velocity = 0

def stateCallback(msg): 
  rospy.loginfo(msg.data) #prints on terminal

  # Controlller takes in boat data from msg that it subscribes to
  
  ################# Implement Controller Here #################
  
  feedback_linear_velocity= msg.data.x
  feedback_angular = msg.data.w
 
  ##############################################################
  

def callback2(msg): 
  rospy.loginfo(msg.data) #prints on terminal

  # Controlller takes in boat data from msg that it subscribes to
  
  ################# Implement Controller Here #################
  
  pub.publish(move)
  
  
  

rospy.init_node('controller_node')

sub = rospy.Subscriber('/mavros/velocity_unstamped', Twist, stateCallback) #We subscribe to the boats velocity
sub = rospy.Subscriber('/mavros', Twist, commandCallback) #We subscribe to the boats velocity

pub = rospy.Publisher('/controller/output_pwm', Twist, queue_size=2)
rate = rospy.Rate(2)


rospy.spin()
