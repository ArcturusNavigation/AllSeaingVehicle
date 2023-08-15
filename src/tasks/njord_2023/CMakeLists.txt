cmake_minimum_required(VERSION 3.0.2)
project(njord_2023)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  rospy
  std_msgs
)
roslaunch_add_file_check(launch)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################

catkin_package()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	src/maneuver.py
)

