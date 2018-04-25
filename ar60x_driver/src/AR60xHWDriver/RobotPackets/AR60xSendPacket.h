#ifndef AR60XSENDPACKET_H
#define AR60XSENDPACKET_H

#include "AR60xPacketsDefinitions.h"
#include "BasePacket.h"
#include <DataTypes/SensorFeetState.h>
#include <DataTypes/PowerSources.h>
#include <RobotDescription/AR60xDescription.h>
#include <iostream>
#include <map>
#include <bitset>
#include <cstdlib>
#include <cstring>
#include <ros/ros.h>
#include <robot_msgs/JointsCommand.h>
#include <robot_msgs/TypePid.h>
#include <robot_msgs/JointsParams.h>
#include <robot_msgs/JointsMode.h>
#include <sensor_msgs/Imu.h>


class AR60xSendPacket : public BasePacket
{
public:
    AR60xSendPacket(AR60xDescription& robotDesc);

    void JointSetPosition(JointData &joint, double value);
    void JointSetPIDGains(JointData &joint, robot_msgs::TypePid gains);
    void JointSetReverse(JointData &joint, bool reverse);
    void JointSetLimits(JointData &joint, double lower, double upper);
    void JointSetOffset(JointData &joint, double value);
    void JointSetMode(JointData &joint, robot_msgs::TypeJointMode mode);

    void PowerSourceSetOnOff(PowerSources supply, bool onOffState);

    void SensorSetImuCalibration(sensor_msgs::Imu imu);
    void SensorSetFeetCalibration(SensorFeetState data);

private:
    void sensorSetOffset(uint8_t groupId, uint8_t number, double value);
    void sensorSetFootOffset(SensorFeetState::FootData data, uint8_t groupId);
    void write_int16(uint16_t address, int16_t value);
    short angle_to_uint16(double angle);
    double rad2deg(double rad);

};

#endif // AR60XSENDPACKET_H
