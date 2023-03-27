#!/usr/bin/env python
from collections import namedtuple

import rospy 
import numpy as np 
from std_msgs.msg import String
from geometry_msgs.msg import Twist 
from pilot_suite.msg import VelocityCommand
# from task_node import TaskNode
from pilot_suite.task_node import TaskNode
from pilot_suite.object_types.roboboat import Label