cmake_minimum_required(VERSION 2.8.3)
project(robot-controller-ros)

# Tell CMake to run moc when necessary:
set(CMAKE_AUTOMOC ON)
# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set (CMAKE_CXX_STANDARD 11)

## Find packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  roslib
)

find_package (Qt5Core)
find_package (Qt5Network)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES robot-controller-ros
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs roslib
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#aux_source_directory(src/AR60x_driver AR60X_DRIVER_SOURCES)
add_executable(AR60x_driver
        src/AR60x_driver/main.cpp
        src/AR60x_driver/AR60xHWDriver/AR60xHWDriver.cpp
        src/AR60x_driver/AR60xHWDriver/XMLSerializer.cpp
        src/AR60x_driver/AR60xHWDriver/UDPConnection.cpp
        src/AR60x_driver/AR60xHWDriver/DataTypes/ConnectionData.cpp
        src/AR60x_driver/AR60xHWDriver/DataTypes/JointData.cpp
        src/AR60x_driver/AR60xHWDriver/DataTypes/JointState.cpp
        src/AR60x_driver/AR60xHWDriver/DataTypes/PowerData.cpp
        src/AR60x_driver/AR60xHWDriver/DataTypes/PowerState.cpp
        src/AR60x_driver/AR60xHWDriver/DataTypes/SensorData.cpp
        src/AR60x_driver/AR60xHWDriver/DataTypes/SensorState.cpp
        src/AR60x_driver/AR60xHWDriver/Libs/TinyXML2/tinyxml2.cpp
        src/AR60x_driver/AR60xHWDriver/RobotDescription/AR60xDescription.cpp
        src/AR60x_driver/AR60xHWDriver/RobotPackets/AR60xRecvPacket.cpp
        src/AR60x_driver/AR60xHWDriver/RobotPackets/AR60xSendPacket.cpp)

target_include_directories(AR60x_driver PUBLIC
                               ${Qt5Core_INCLUDE_DIRS}
                               ${Qt5Network_INCLUDE_DIRS})

target_link_libraries(AR60x_driver
                          ${catkin_LIBRARIES}
                          ${Qt5Core_LIBRARIES}
                          ${Qt5Network_LIBRARIES})
