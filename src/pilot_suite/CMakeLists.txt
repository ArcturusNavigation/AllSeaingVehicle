cmake_minimum_required(VERSION 3.0.2)
project(pilot_suite)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	mavros
	message_generation
	roslaunch
)
roslaunch_add_file_check(launch)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	RawWaypoint.msg
	ProcessedWaypoint.msg
	SkipWaypoint.msg
	ProcessedTask.msg
	WaypointReached.msg
	VelocityCommand.msg
)

## Generate services in the 'srv' folder
add_service_files(
	FILES
	GoToWaypoint.srv
	PwmCommand.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    geographic_msgs
)

###########
## Build ##
###########

include_directories(
	${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
	scripts/ardupilot.py
	scripts/navigation_pilot.py
	scripts/roboboat_planner.py
	scripts/waypoint_sim.py
	scripts/tf2.py
	scripts/local_planner.py
	scripts/mode_listener.py
	scripts/turtle_nest.py
	scripts/pilot_shifter.py
	scripts/open_loop_distance.py
	scripts/velocity_control.py
	scripts/pwm_sender.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
