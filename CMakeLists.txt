cmake_minimum_required(VERSION 2.8.3)
project(robot_controller_ros)

# Tell CMake to run moc when necessary:
set(CMAKE_AUTOMOC ON)
# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set (CMAKE_CXX_STANDARD 14)

## Find packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  roslib
  message_generation
)


add_message_files(
        FILES
        TypeFootSensor.msg
        TypeJointMode.msg
        TypePid.msg
        TypeSupplyState.msg
        FeetSensors.msg
        JointsCommand.msg
        JointsParams.msg
        JointsMode.msg
        JointsSupplyState.msg
        SourcesSupplyState.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot-controller-ros
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs roslib message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_subdirectory(src/AR60xHWDriver)
add_subdirectory(src/AR60x_driver)