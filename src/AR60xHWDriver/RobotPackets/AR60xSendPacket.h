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
#include <robot_controller_ros/Pid.h>
#include <robot_controller_ros/JointsParams.h>
#include <robot_controller_ros/JointsMode.h>


class AR60xSendPacket : public BasePacket
{
public:
    AR60xSendPacket(AR60xDescription& robotDesc);



    void jointSetState(uint8_t number, JointState state );
    void supplySetOn( PowerData::PowerSupplies supply );
    void supplySetOff( PowerData::PowerSupplies supply );
    void sensorSetOffset(uint8_t groupId, uint8_t number, double value);
    void sensorSetImuOffset(SensorImuState data);
    void sensorSetFeetOffset(SensorFeetState data);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Send position command to joints
     *
     * @param command ROS message with joints names, positions and PID-gains
     */
    void JointsSetPosition(robot_controller_ros::JointsCommand command);

    /**
     * Setup joints params (limits, offset, reverse)
     * @param params ROS message with joints params
     */
    void JointsSetParams(robot_controller_ros::JointsParams params);

    /**
     * Control joint's motor and clutch
     * @param mode ROS message with  joint's operational mode
     */
    void JointsSetMode(robot_controller_ros::JointsMode mode);

    /**
    * Set supply source on/off
    * @param supply Selected supply
    * @param onOffState On or off
    */
    void SupplySetOnOff(PowerData::PowerSupplies supply, bool onOffState);

private:
    void jointSetPosition(JointData& joint, double value);
    void jointSetPIDGains(JointData& joint, robot_controller_ros::Pid gains);
    void jointSetLimits(JointData& joint, double lower, double upper)
    void jointSetOffset(JointData& joint, double value);
    void jointSetReverse(JointData& joint, bool reverse);

    void sensorSetFootOffset(SensorFeetState::FootData data, uint8_t groupId);
    void write_int16(uint16_t address, int16_t value);
    short angle_to_uint16(double angle);

};

#endif // AR60XSENDPACKET_H
