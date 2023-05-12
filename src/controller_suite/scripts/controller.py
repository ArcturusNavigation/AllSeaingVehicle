#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist

disToObstacle = 1

def callback(msg): 
  rospy.loginfo(msg.data) #prints on terminal

  # Controlller takes in boat data from msg that it subscribes to
  
  ################# Implement Controller Here #################
  
  
  
  
  
  
  ##############################################################
  #Some PWM Command
  pwm = 100
  pub.publish(move)
  

rospy.init_node('controller_node')
sub = rospy.Subscriber('/mavros', Twist, callback) #We subscribe to the boats velocity
pub = rospy.Publisher('/controller/output_pwm', Twist, queue_size=2)
rate = rospy.Rate(2)
move = Twist()


rospy.spin()
