#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist

disToObstacle = 1

def callback(msg): 
  rospy.loginfo(msg.data) #prints on terminal

  # Controlller takes in the information from the boudning boxes from msg that it subscribes to
  
  ################# Implement Controller from Bounding Box to Velocity Here #################
  
  
  
  
  
  
  ##############################################################
  #Some PWM Command
  pwm = 100
  pub.publish(move)
  

rospy.init_node('visual_servoing_node')
sub = rospy.Subscriber('/perception/bounding_box_stuff', Twist, callback) #We subscribe to the boats velocity
pub = rospy.Publisher('/visual_servoing/velocity', Twist, queue_size=2)
rate = rospy.Rate(2)
move = Twist()


rospy.spin()
