cmake_minimum_required(VERSION 3.0.2)
project(perception_suite)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	genmsg
	geometry_msgs
	image_geometry
	laser_geometry
	message_generation
	pcl_ros
	roscpp
	rospy
	sensor_msgs
	std_msgs
	visualization_msgs
)

find_package(OpenCV REQUIRED)

add_message_files(
	FILES
	LabeledBoundingBox2D.msg
	LabeledBoundingBox2DArray.msg
	Object.msg
	ObjectArray.msg
)

generate_messages(
	DEPENDENCIES
	sensor_msgs
	geometry_msgs 
)

catkin_package(
	INCLUDE_DIRS include
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${CUDA_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
	scripts/yolov5_detector.py
	scripts/yolov5_image.py
	scripts/buoy_detector.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

link_directories(${CUDA_LIBRARY_DIRS})

add_executable(segmentation src/segmentation_node.cpp)
target_link_libraries(segmentation ${OpenCV_LIBRARIES})
target_link_libraries(segmentation ${catkin_LIBRARIES})
add_dependencies(segmentation perception_suite_generate_messages_cpp)

add_executable(processor src/processor_nodes.cpp)
target_link_libraries(processor ${catkin_LIBRARIES})
add_dependencies(processor perception_suite_generate_messages_cpp)
