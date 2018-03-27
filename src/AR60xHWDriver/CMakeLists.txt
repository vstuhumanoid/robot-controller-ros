project(AR60xHWDriver)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        cmake_modules)
find_package(TinyXML2 REQUIRED)

add_library(${PROJECT_NAME} STATIC
        AR60xHWDriver.cpp
        XMLSerializer/XMLSerializer.cpp
        UDPConnection/UDPConnection.cpp
        DataTypes/ConnectionData.h
        DataTypes/SensorImuState.h
        DataTypes/JointData.h
        DataTypes/JointState.h
        DataTypes/SensorFeetState.h
        DataTypes/PowerData.h
        DataTypes/PowerState.h
        DataTypes/SensorData.h
        DataTypes/SensorsGroupData.h
        RobotPackets/BasePacket.cpp
        RobotPackets/AR60xRecvPacket.cpp
        RobotPackets/AR60xSendPacket.cpp
        RobotPackets/AR60xPacketsDefinitions.h)


target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${TinyXML2_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})
