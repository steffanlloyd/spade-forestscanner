cmake_minimum_required(VERSION 2.8.3)
project(spade)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  livox_ros_driver2
  livox_to_pointcloud2
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES spade
  CATKIN_DEPENDS roscpp livox_ros_driver2 livox_to_pointcloud2
  #DEPENDS system_lib
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
