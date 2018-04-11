project(AR60xHWDriver)

set (CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        cmake_modules)
find_package(TinyXML2 REQUIRED)

add_library(${PROJECT_NAME} STATIC
        AR60xHWDriver.cpp
        XMLSerializer/XMLSerializer.cpp
        UDPConnection/UDPConnection.cpp
        DataTypes/ConnectionData.h
        DataTypes/PowerSources.h
        DataTypes/SensorFeetState.h
        RobotPackets/BasePacket.cpp
        RobotPackets/AR60xRecvPacket.cpp
        RobotPackets/AR60xSendPacket.cpp
        RobotPackets/AR60xPacketsDefinitions.h
        RobotDescription/JointData.h
        RobotDescription/SensorData.h
        RobotDescription/SensorsGroupData.h
        RobotDescription/AR60xDescription.h)


target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${TinyXML2_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})
