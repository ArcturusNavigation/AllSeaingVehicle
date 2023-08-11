cmake_minimum_required(VERSION 3.0.2)
project(roboboat_2023)

find_package(catkin REQUIRED COMPONENTS
	rospy
	std_msgs
	roslaunch
)
roslaunch_add_file_check(launch)

###################################
## catkin specific configuration ##
###################################

catkin_package()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
	src/ball_launch.py
	src/ocean_cleanup_nav.py
	src/roboboat_planner.py
	src/skeeball_task.py
	src/turtle_nest.py
	src/water_gun_task.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
