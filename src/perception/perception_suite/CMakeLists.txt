cmake_minimum_required(VERSION 3.0.2)
project(perception_suite)

# Compile as C++14(required for pcl)
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LabeledBoundingBox2D.msg
  LabeledBoundingBox2DArray.msg
  Object.msg
  ObjectArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs 
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES perception_suite
# CATKIN_DEPENDS message_runtime #cv_bridge genmsg geometry_msgs image_geometry laser_geometry message_generation pcl_ros roscpp rospy sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/yolov5_detector.py
  scripts/buoy_segmentation.py
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
