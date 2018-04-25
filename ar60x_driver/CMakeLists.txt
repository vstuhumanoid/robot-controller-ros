cmake_minimum_required(VERSION 2.8.3)
project(ar60x_driver)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set (CMAKE_CXX_STANDARD 14)

## Find packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  robot_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot-controller-ros
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_subdirectory(src/AR60xHWDriver)
add_subdirectory(src/ar60x_driver_node)
add_subdirectory(src/topic_converters)