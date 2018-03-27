#ifndef AR60XSENDPACKET_H
#define AR60XSENDPACKET_H

#include "AR60xPacketsDefinitions.h"
#include "BasePacket.h"
#include <DataTypes/JointState.h>
#include <DataTypes/SensorImuState.h>
#include <DataTypes/SensorFeetState.h>
#include <RobotDescription/AR60xDescription.h>

#include <iostream>
#include <map>
#include <stdlib.h>
#include <cstring>
#include <ros/ros.h>
#include <robot_controller_ros/JointsCommand.h>
#include <robot_controller_ros/TypePid.h>
#include <robot_controller_ros/JointsParams.h>
#include <robot_controller_ros/JointsMode.h>


class AR60xSendPacket : public BasePacket
{
public:
    AR60xSendPacket(AR60xDescription& robotDesc);

    void sensorSetImuOffset(SensorImuState data);
    void sensorSetFeetOffset(SensorFeetState data);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //TODO: Sensors' calibration

    void jointSetPosition(JointData& joint, double value);
    void jointSetPIDGains(JointData& joint, robot_controller_ros::TypePid gains);
    void jointSetReverse(JointData& joint, bool reverse);
    void jointSetLimits(JointData& joint, double lower, double upper);
    void jointSetOffset(JointData& joint, double value);
    void jointSetMode(JointData& joint, robot_controller_ros::TypeJointMode mode);
    void supplySetOnOff(PowerData::PowerSupplies supply, bool onOffState);

private:

    void sensorSetFootOffset(SensorFeetState::FootData data, uint8_t groupId);
    void write_int16(uint16_t address, int16_t value);
    short angle_to_uint16(double angle);

};

#endif // AR60XSENDPACKET_H
