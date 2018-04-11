#ifndef AR60XRECVPACKET_H
#define AR60XRECVPACKET_H

#include <iostream>
#include <map>
#include <cstdint>
#include <ros/ros.h>
#include "BasePacket.h"
#include "AR60xPacketsDefinitions.h"
#include <RobotDescription/AR60xDescription.h>
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

    double JointGetPosition(const JointData &joint) const;
    double JointGetLowerLimit(const JointData &joint) const;
    double JointGetUpperLimit(const JointData &joint) const;
    double JointGetOffset(const JointData &joint) const;
    robot_controller_ros::TypeJointMode JointGetMode(const JointData &joint) const;
    robot_controller_ros::TypePid JointGetPidGains(const JointData &joint) const ;

    robot_controller_ros::TypeSupplyState PowerGetJointSupplyState(const JointData &joint) const;
    robot_controller_ros::TypeSupplyState PowerGetSourceSupplyState(const PowerSources supply) const;

    sensor_msgs::Imu SensorsGetImu() const;
    robot_controller_ros::FeetSensors SensorsGetFeet() const;

private:
    robot_controller_ros::TypeFootSensor sensorGetFoot(const uint8_t groupId) const;

    int16_t read_int16(const uint16_t address) const;
    float read_float(const uint16_t address) const;
    double int16_to_angle(const int16_t angle) const;
};

#endif // AR60XRECVPACKET_H
