#!/usr/bin/env python
from collections import namedtuple

import rospy 
import numpy as np 
from geometry_msgs.msg import Twist 
from pilot_suite.msg import VelocityCommand
from pilot_suite.task_node import TaskNode

from sensor_suite.msg import LabeledBoundingBox2DArray, LabeledBoundingBox2D

class BuoyNavNode(TaskNode):
    def __init__(self,img_height, img_width):
        super().__init__("buoy_tasks")
        self.bbox_sub = rospy.Subscriber("/sensor_suite/bounding_boxes", LabeledBoundingBox2DArray, self.nav_callback)
        self.pub = rospy.Publisher("/pilot_suite/velocity_command", VelocityCommand, queue_size=10)
        self.img_height = img_height
        self.img_width = img_width

    def nav_callback(self, msg):
        if not self.active:
            return
        port_pos = 0 # Left objects 
        starboard_pos = self.img_width #Right objects
        obstacle_pos = -1 # Only used if we find obstacles
        # We use the object closest to the bottom of the image i.e. closest to the boat
        port_y = 0 
        starboard_y = 0
        obstacle_y = 0
        for box in msg.boxes:
            box_pos_x = (box.max_x + box.min_x)/2
            box_pos_y = (box.max_y + box.min_y)/2 
            if box.label  in [Label.RED_POLE.value, Label.RED_BUOY.value]:
                if box_pos_x < self.img_width/2:
                    if box_pos_y > port_y:
                        port_pos = box_pos_x
                        port_y = box_pos_y
                else:
                    #TODO: Figure out what to do in this case
                    print(f"WARNING: Red object found but not on port side ({(box_pos_x,box_pos_y)})")
            elif box.label in [Label.GREEN_POLE.value,Label.GREEN_BUOY.value]:
                if box_pos_x > self.img_width/2: 
                    if box_pos_y > starboard_y:
                        starboard_pos = box_pos_x
                        starboard_y = box_pos_y
                else:
                    #TODO: Figure out what to do in this case
                    print(f"WARNING: Green object found but not on starboard side ({(box_pos_x,box_pos_y)})")
            elif box.label in [Label.YELLOW_BUOY.value, Label.BLACK_BUOY.value, Label.BLUE_BUOY.value]:
                if box_pos_y > obstacle_y:
                    obstacle_pos = box_pos_x
                    obstacle_y = box_pos_y
        if obstacle_pos == -1:
            midpoint_x = (port_pos + starboard_pos)/2.0
        elif obstacle_pos < self.img_width/2:
            midpoint_x = (port_pos + obstacle_pos)/2.0
        else:
            midpoint_x = (starboard_pos + obstacle_pos)/2.0
        command_angle = np.arctan2(midpoint_x - self.img_width/2, self.img_height)
        vel_command = VelocityCommand() 
        command = Twist()
        command.angular.z = command_angle
        command.linear.x = 10.0
        vel_command.twist = command
        vel_command.cancel = False
        self.pub.publish(vel_command)

if __name__ == "__main__":
    try:
        rospy.init_node("navigation_pilot")
        BuoyNavNode(480, 640) #TODO: Make these parameters
        rospy.spin()   
    except rospy.ROSInterruptException:
        pass   

