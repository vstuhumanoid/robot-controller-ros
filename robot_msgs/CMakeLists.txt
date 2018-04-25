cmake_minimum_required(VERSION 2.8.3)
project(robot_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
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

catkin_package(
        CATKIN_DEPENDS sensor_msgs std_msgs message_runtime
)