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
	scripts/ball_launch.py
	scripts/ocean_cleanup_nav.py
	scripts/roboboat_planner.py
	scripts/skeeball_task.py
	scripts/turtle_nest.py
	scripts/water_gun_task.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
