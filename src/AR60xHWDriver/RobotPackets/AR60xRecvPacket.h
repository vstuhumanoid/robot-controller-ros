#ifndef AR60XRECVPACKET_H
#define AR60XRECVPACKET_H

#include <iostream>
#include <map>
#include <cstdint>
#include <ros/ros.h>

#include "BasePacket.h"
#include "AR60xPacketsDefinitions.h"

#include <RobotDescription/AR60xDescription.h>
#include <DataTypes/SensorFeetState.h>
#include <DataTypes/PowerSources.h>
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

    double JointGetPosition(JointData &joint);
    double JointGetLowerLimit(JointData &joint);
    double JointGetUpperLimit(JointData &joint);
    double JointGetOffset(JointData &joint);
    robot_controller_ros::TypeJointMode JointGetMode(JointData &joint);
    robot_controller_ros::TypePid JointGetPidGains(JointData &joint);

    robot_controller_ros::TypeSupplyState PowerGetJointSupplyState(JointData &joint);
    robot_controller_ros::TypeSupplyState PowerGetSourceSupplyState(PowerSources supply);

    sensor_msgs::Imu SensorsGetImu();
    SensorFeetState SensorsGetFeet();

private:
    SensorFeetState::FootData sensorGetFoot(uint8_t groupId);

    int16_t read_int16(uint16_t address);
    float read_float(uint16_t address);
    double int16_to_angle(int16_t angle);
};

#endif // AR60XRECVPACKET_H
