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


void AR60xSendPacket::jointSetPosition(uint8_t number, double value)
{
    double lowerLimit = desc_.joints[number].limits.lowerLimit;
    double upperLimit = desc_.joints[number].limits.upperLimit;
    bool isReverse = desc_.joints[number].isReverse;
    uint8_t channel = desc_.joints[number].channel;

    if(value < lowerLimit)
        value = lowerLimit;
    else if(value > upperLimit)
        value = upperLimit;

    if(isReverse)
        value = -value;

    write_int16(channel * 16 + JointPositionAddress, angle_to_uint16(value));
}

void AR60xSendPacket::jointSetOffset(uint8_t number, double value)
{
    short channel = desc_.joints[number].channel;
    write_int16(channel * 16 + JointOffsetAddress, angle_to_uint16(value));
}


void AR60xSendPacket::jointSetPIDGains(uint8_t number, JointData::PIDGains gains)
{
    short channel = desc_.joints[number].channel;
    write_int16(channel * 16 + JointPGainAddress, gains.proportional);
    write_int16(channel * 16 + JointIGainAddress, gains.integral);
    write_int16(channel * 16 + JointDGainAddress, gains.derivative);
}

void AR60xSendPacket::jointSetLowerLimit(uint8_t number, double value)
{
    short channel = desc_.joints[number].channel;
    bool isReverce = desc_.joints[number].isReverse;

    if(isReverce)
        write_int16(channel * 16 + JointUpperLimitAddress, angle_to_uint16(-value));
    else
        write_int16(channel * 16 + JointLowerLimitAddress, angle_to_uint16(value));
}

void AR60xSendPacket::jointSetUpperLimit(uint8_t number, double value)
{
    short channel = desc_.joints[number].channel;
    bool isReverce = desc_.joints[number].isReverse;

    if(isReverce)
        write_int16(channel * 16 + JointLowerLimitAddress, angle_to_uint16(-value));
    else
        write_int16(channel * 16 + JointUpperLimitAddress, angle_to_uint16(value));
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

void AR60xSendPacket::write_int16(uint16_t address, int16_t value)
{
    byte_array_[address + 1] = (BYTE)(value >> 8);
    byte_array_[address] = value;
}

short AR60xSendPacket::angle_to_uint16(double angle)
{
    return (short)(angle * 100);
}