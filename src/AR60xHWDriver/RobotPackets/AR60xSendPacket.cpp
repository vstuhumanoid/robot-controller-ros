#include "AR60xSendPacket.h"

AR60xSendPacket::AR60xSendPacket(AR60xDescription& robotDesc) : BasePacket(robotDesc)
{
    memset(byte_array_, 0, packetSize);

    for (auto &it : desc_.joints)
    {
        JointData joint = it.second;
        write_int16(joint.channel * 16, joint.number);
        jointSetPIDGains(joint.number, joint.gains);
        jointSetOffset(joint.number, joint.offset);

        jointSetLowerLimit(joint.number, joint.limits.lowerLimit);
        jointSetUpperLimit(joint.number, joint.limits.upperLimit);

        JointState state;
        state.state = JointState::MotorState::STOP;
        state.controlType = JointState::ControlType::POSITION_CONTROl;
        jointSetState(joint.number, state);
    }

    for(auto& sensorGroup: desc_.sensorGroups)
        for(auto& sensor: sensorGroup.second.sensors)
            sensorSetOffset(sensorGroup.second.id, sensor.number, sensor.offset);
}






void AR60xSendPacket::jointSetState(uint8_t number, JointState state)
{
    short channel = desc_.joints[number].channel;
    bool isEnable = desc_.joints[number].isEnable;
    uint8_t* status = &byte_array_[channel * 16 + 1];

    // shit bit magic
    switch (state.state)
    {
        case JointState::MotorState::BRAKE:
            *status |= 0b00;
            break;
        case JointState::MotorState::STOP:
            *status |= 0b01;
            break;
        case JointState::MotorState::RELAX:
            *status |= 0b10;
            break;
        case JointState::MotorState::TRACE:
            if(isEnable) *status |= 0b11;
            break;
        default:
            break;
    }

    if(state.controlType == JointState::ControlType::POSITION_CONTROl)
        *status &= ~(1 << 7);
    else if(state.controlType == JointState::ControlType::TORQUE_CONTROL)
        *status |= (1 << 7);
}

void AR60xSendPacket::supplySetOff(PowerData::PowerSupplies supply)
{
    //TODO: Check invalid value. Invalid value casted to enum = UB
    byte_array_[PowerDataAddress + 1] &= (255 - (1 << supply));

}

void AR60xSendPacket::supplySetOn(PowerData::PowerSupplies supply)
{
    //TODO: Check invalid value. Invalid value casted to enum = UB
    byte_array_[PowerDataAddress + 1] |= 1 << supply;
}

void AR60xSendPacket::sensorSetOffset(uint8_t groupId, uint8_t number, double value)
{
    uint8_t channel = desc_.sensorGroups[groupId].channel;
    write_int16(channel * 16 + sensorsMap[number], value * 100);
}

void AR60xSendPacket::sensorSetImuOffset(SensorImuState data)
{
    uint8_t channel = desc_.sensorGroups[ImuGroupId].channel;
    write_int16(channel * 16 + SensorAccXOffset, data.accX * 100);
    write_int16(channel * 16 + SensorAccYOffset, data.accY * 100);
    write_int16(channel * 16 + SensorAccZOffset, data.accZ * 100);
    write_int16(channel * 16 + SensorYawOffset, data.yaw * 100);
    write_int16(channel * 16 + SensorPitchOffset, data.pitch * 100);
    write_int16(channel * 16 + SensorRollOffset, data.roll * 100);
}

void AR60xSendPacket::sensorSetFeetOffset(SensorFeetState data)
{
    sensorSetFootOffset(data.left, LeftFootGroupId);
    sensorSetFootOffset(data.right, RightFootGroupId);
}

void AR60xSendPacket::sensorSetFootOffset(SensorFeetState::FootData data, uint8_t groupId)
{
    uint8_t channel = desc_.sensorGroups[groupId].channel;
    write_int16(channel * 16 + SensorUch0Offset, data.uch0 * 100);
    write_int16(channel * 16 + SensorUch1Offset, data.uch1 * 100);
    write_int16(channel * 16 + SensorUch2Offset, data.uch2 * 100);
    write_int16(channel * 16 + SensorUch3Offset, data.uch3 * 100);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void AR60xSendPacket::JointsSetPosition(robot_controller_ros::JointsCommand command)
{
    if((command.names.size() != command.positions.size()) && (command.pids.size() == 0 || command.pids.size() != command.names.size()))
    {
        ROS_ERROR_STREAM("JointsSetCommand: names and positions vectors should be same size. pids should be zero or same size");
        ROS_ERROR_STREAM("Ignoring command");
        return;
    }

    for(int i = 0; i < command.names.size(); i++)
    {
        JointData& joint = desc_.joints[atoi(command.names[i].c_str())];
        jointSetPosition(joint, command.positions[i]);

        if(command.pids.size() > 0)
            jointSetPIDGains(joint, command.pids[i]);
    }
}

void AR60xSendPacket::JointsSetParams(robot_controller_ros::JointsParams params)
{
    if(params.names.size() != params.params.size())
    {
        ROS_ERROR_STREAM("JointSetParams: names and params vectors should be same size");
        ROS_ERROR_STREAM("Ignoring command");
        return;
    }

    for(int i = 0; i < params.names.size(); i++)
    {
        JointData& joint = desc_.joints[atoi(params.names[i].c_str())];

        jointSetReverse(joint, params.params[i].reverse);
        jointSetLimits(joint, params.params[i].lower_limit, params.params[i].upper_limit);
        jointSetOffset(joint, params.params[i].offset);
    }
}

void AR60xSendPacket::JointsSetMode(robot_controller_ros::JointsMode mode)
{
    if(mode.names.size() != mode.modes.size())
    {
        ROS_ERROR_STREAM("JointSetParams: names and modes vectors should be same size");
        ROS_ERROR_STREAM("Ignoring command");
        return;
    }

    for(int i = 0; i<mode.names.size(); i++)
    {
        JointData& joint = desc_.joints[atoi(mode.names[i].c_str())];

    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void AR60xSendPacket::jointSetPosition(JointData& joint, double value)
{
    if(value < joint.lower_limit)
        value = joint.lower_limit;
    else if(value > joint.upper_limit)
        value = joint.upper_limit;

    if(joint.isReverse)
        value = -value;

    write_int16(joint.channel * 16 + JointPositionAddress, angle_to_uint16(value));
}

void AR60xSendPacket::jointSetPIDGains(JointData& joint, robot_controller_ros::Pid gains)
{
    joint.gains = gains;

    short channel = joint.channel;
    write_int16(channel * 16 + JointPGainAddress, gains.p);
    write_int16(channel * 16 + JointIGainAddress, gains.i);
    write_int16(channel * 16 + JointDGainAddress, gains.d);
}

void AR60xSendPacket::jointSetReverse(JointData &joint, bool reverse)
{
    joint.isReverse = reverse;
}

void AR60xSendPacket::jointSetLimits(JointData& joint, double lower, double upper)
{
    joint.lower_limit = lower;
    joint.upper_limit = upper;

    short channel = joint.channel;

    if(joint.isReverse)
    {
        write_int16(channel * 16 + JointUpperLimitAddress, angle_to_uint16(-lower));
        write_int16(channel * 16 + JointLowerLimitAddress, angle_to_uint16(-upper));
    }
    else
    {
        write_int16(channel * 16 + JointLowerLimitAddress, angle_to_uint16(lower));
        write_int16(channel * 16 + JointUpperLimitAddress, angle_to_uint16(upper));
    }
}

void AR60xSendPacket::jointSetOffset(JointData& joint, double value)
{
    write_int16(joint.channel * 16 + JointOffsetAddress, angle_to_uint16(value));
}


void AR60xSendPacket::jointSetMode(JointData& joint)
{
    
}

void AR60xSendPacket::write_int16(uint16_t address, int16_t value)
{
    byte_array_[address + 1] = (BYTE)(value >> 8);
    byte_array_[address] = value;
}

short AR60xSendPacket::angle_to_uint16(double angle)
{
    return (short)(angle * 100);
}



