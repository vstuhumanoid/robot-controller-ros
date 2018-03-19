project(AR60xHWDriver)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        cmake_modules)
find_package(TinyXML2 REQUIRED)

find_package (Qt5Core REQUIRED)
find_package (Qt5Network REQUIRED)

add_library(${PROJECT_NAME} STATIC
        AR60xHWDriver.cpp
        XMLSerializer.cpp
        UDPConnection.cpp
        DataTypes/ConnectionData.cpp
        DataTypes/JointData.cpp
        DataTypes/JointState.cpp
        DataTypes/PowerData.cpp
        DataTypes/PowerState.cpp
        DataTypes/SensorData.cpp
        DataTypes/SensorState.cpp
        RobotDescription/AR60xDescription.cpp
        RobotPackets/AR60xRecvPacket.cpp
        RobotPackets/AR60xSendPacket.cpp
        HardwareInterfaces/IAR60xJointControl.h
        HardwareInterfaces/IAR60xJointState.h
        HardwareInterfaces/IAR60xPowerControl.h
        HardwareInterfaces/IAR60xSensorState.h
        HardwareInterfaces/IAR60xSensorState.h)


target_link_libraries(${PROJECT_NAME}
        ${Qt5Core_LIBRARIES}
        ${Qt5Network_LIBRARIES}
        ${catkin_LIBRARIES}
        ${TinyXML2_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})
