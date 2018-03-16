cmake_minimum_required(VERSION 2.8.3)
project(robot-controller-ros)

set (CMAKE_CXX_STANDARD 11)

## Find packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

find_package (Qt5Core)
find_package (Qt5Network)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot-controller-ros
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(AR60x_driver src/AR60x_driver/main.cpp
                            src/AR60x_driver/AR60xHWDriver)
target_include_directories(AR60x_driver PUBLIC
                                        ${Qt5Core_INCLUDE_DIRS}
                                        ${Qt5Network_INCLUDE_DIRS})
target_link_libraries(AR60x_driver ${catkin_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Network_LIBRARIES})
