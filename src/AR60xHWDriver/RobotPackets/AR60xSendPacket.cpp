#include "AR60xSendPacket.h"

AR60xSendPacket::AR60xSendPacket(AR60xDescription& robotDesc) : BasePacket(robotDesc)
{
    memset(byte_array_, 0, packetSize);

    for (auto &it : desc_.joints)
    {
        JointData& joint = it.second;
        write_int16(joint.channel * 16, joint.number);

        JointSetPIDGains(joint, joint.pid_gains);
        JointSetOffset(joint, joint.offset);
        JointSetLimits(joint, joint.lower_limit, joint.upper_limit);
        JointSetPIDGains(joint, joint.pid_gains);

        robot_controller_ros::TypeJointMode mode;
        mode.mode = robot_controller_ros::TypeJointMode::BREAK;
        JointSetMode(joint, mode);
    }

    for(auto& sensorGroup: desc_.sensorGroups)
    {
        for (auto &sensor: sensorGroup.second.sensors)
        {
            write_int16(sensorGroup.second.channel * 16, sensorGroup.second.channel);
            sensorSetOffset(sensorGroup.second.id, sensor.number, sensor.offset);
        }
    }


    PowerSourceSetOnOff(PowerSources::Supply12V, true);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void AR60xSendPacket::JointSetPosition(JointData &joint, double value)
{
    value = rad2deg(value);
    //WARNING: limits in degrees.

    if(value < joint.lower_limit)
        value = joint.lower_limit;
    else if(value > joint.upper_limit)
        value = joint.upper_limit;

    if(joint.is_reverse)
        value = -value;

    write_int16(joint.channel * 16 + JointPositionAddress, angle_to_uint16(value));
}

void AR60xSendPacket::JointSetPIDGains(JointData &joint, robot_controller_ros::TypePid gains)
{
    joint.pid_gains = gains;

    short channel = joint.channel;
    write_int16(channel * 16 + JointPGainAddress, gains.p);
    write_int16(channel * 16 + JointIGainAddress, gains.i);
    write_int16(channel * 16 + JointDGainAddress, gains.d);
}

void AR60xSendPacket::JointSetReverse(JointData &joint, bool reverse)
{
    joint.is_reverse = reverse;
}

void AR60xSendPacket::JointSetLimits(JointData &joint, double lower, double upper)
{
    //TODO: rad2deg
    joint.lower_limit = lower;
    joint.upper_limit = upper;

    short channel = joint.channel;

    if(joint.is_reverse)
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

void AR60xSendPacket::JointSetOffset(JointData &joint, double value)
{
    //TODO: rad2deg
    joint.offset = value;
    write_int16(joint.channel * 16 + JointOffsetAddress, angle_to_uint16(value));
}


void AR60xSendPacket::JointSetMode(JointData &joint, robot_controller_ros::TypeJointMode mode)
{
    uint8_t* status = &byte_array_[joint.channel * 16 + 1];

    switch (mode.mode)
    {
        case TypeJointMode::BREAK:
            *status &= ~0b11; //set 0b00
            break;
        case TypeJointMode::STOP:
            *status &= ~0b10; //set 0b01
            *status |= 0b01;
            break;
        case TypeJointMode::RELAX:
            *status |= 0b10;  //set 0b10
            *status &=~0b01;
            break;
        case TypeJointMode::TRACE:
            if(joint.is_enable)
                *status |= 0b11;
            break;
        default:
            break;
    }

    // TODO: Control type

    /*if(state.controlType == JointState::ControlType::POSITION_CONTROl)
        *status &= ~(1 << 7);
    else if(state.controlType == JointState::ControlType::TORQUE_CONTROL)
        *status |= (1 << 7);*/
}

void AR60xSendPacket::PowerSourceSetOnOff(PowerSources supply, bool onOffState)
{
    //TODO: Check invalid value. Invalid value casted to enum = UB

    int a = 1 << static_cast<uint8_t>(supply);

    if(onOffState)
        byte_array_[PowerDataAddress + 1] |= 1 << static_cast<uint8_t>(supply);
    else
        byte_array_[PowerDataAddress + 1] &= ~(1 << static_cast<uint8_t>(supply));
}

void AR60xSendPacket::SensorSetImuCalibration(sensor_msgs::Imu data)
{
    uint8_t channel = desc_.sensorGroups[ImuGroupId].channel;
    write_int16(channel * 16 + SensorAccXOffset, data.linear_acceleration.x * 100);
    write_int16(channel * 16 + SensorAccYOffset, data.linear_acceleration.y * 100);
    write_int16(channel * 16 + SensorAccZOffset, data.linear_acceleration.z * 100);
    write_int16(channel * 16 + SensorYawOffset, data.orientation.z * 100);
    write_int16(channel * 16 + SensorPitchOffset, data.orientation.y * 100);
    write_int16(channel * 16 + SensorRollOffset, data.orientation.x * 100);
}

void AR60xSendPacket::SensorSetFeetCalibration(SensorFeetState data)
{
    sensorSetFootOffset(data.left, LeftFootGroupId);
    sensorSetFootOffset(data.right, RightFootGroupId);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void AR60xSendPacket::sensorSetOffset(uint8_t groupId, uint8_t number, double value)
{
    uint8_t channel = desc_.sensorGroups[groupId].channel;
    write_int16(channel * 16 + sensorsMap[number], value * 100);
}

void AR60xSendPacket::sensorSetFootOffset(SensorFeetState::FootData data, uint8_t groupId)
{
    uint8_t channel = desc_.sensorGroups[groupId].channel;
    write_int16(channel * 16 + SensorUch0Offset, data.uch0 * 100);
    write_int16(channel * 16 + SensorUch1Offset, data.uch1 * 100);
    write_int16(channel * 16 + SensorUch2Offset, data.uch2 * 100);
    write_int16(channel * 16 + SensorUch3Offset, data.uch3 * 100);
}



void AR60xSendPacket::write_int16(uint16_t address, int16_t value)
{
    byte_array_[address + 1] = (uint8_t)(value >> 8);
    byte_array_[address] = value;
}

short AR60xSendPacket::angle_to_uint16(double angle)
{
    return (short)(angle * 100);
}

double AR60xSendPacket::rad2deg(double rad)
{
    return rad / M_PI * 180;
}

