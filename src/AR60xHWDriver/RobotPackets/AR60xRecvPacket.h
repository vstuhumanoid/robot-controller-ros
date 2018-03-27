#ifndef AR60XRECVPACKET_H
#define AR60XRECVPACKET_H

#include <iostream>
#include <map>
#include <cstdint>
#include <ros/ros.h>

#include "BasePacket.h"
#include "AR60xPacketsDefinitions.h"
#include <RobotDescription/AR60xDescription.h>
#include <DataTypes/JointState.h>
#include <DataTypes/SensorImuState.h>
#include <DataTypes/SensorFeetState.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <robot_controller_ros/JointsParams.h>
#include <robot_controller_ros/JointsSupplyState.h>
#include <robot_controller_ros/SourcesSupplyState.h>
#include <robot_controller_ros/FeetSensors.h>

class AR60xRecvPacket : public BasePacket
{
public:
    AR60xRecvPacket(AR60xDescription& robotDesc);
    void initFromByteArray(const uint8_t *bytes);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    sensor_msgs::JointState JointsGetState();
    robot_controller_ros::JointsParams JointsGetParams();
    robot_controller_ros::JointsSupplyState PowerGetJointsSupplyState();
    robot_controller_ros::SourcesSupplyState PowerGetSourcesSupplyState();
    sensor_msgs::Imu SensorsGetImu();
    robot_controller_ros::FeetSensors SensorsGetFeet();

private:

    double jointGetLowerLimit(JointData& joint);
    double jointGetUpperLimit(JointData& joint);
    robot_controller_ros::TypeJointMode jointGetMode(JointData& joint);
    robot_controller_ros::TypeSupplyState jointGetSupplyState(JointData& joint);
    robot_controller_ros::TypeSupplyState sourceGetSupplyState(PowerData::PowerSupplies supply);

    SensorFeetState::FootData sensorGetFoot(uint8_t groupId); // depreacted

    int16_t read_int16(uint16_t address);
    float read_float(uint16_t address);
    double uint16_to_angle(uint16_t angle);
};

#endif // AR60XRECVPACKET_H
