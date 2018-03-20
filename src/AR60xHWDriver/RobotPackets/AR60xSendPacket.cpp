#include "AR60xSendPacket.h"

AR60xSendPacket::AR60xSendPacket(AR60xDescription *robotDesc) : BasePacket(robotDesc)
{
}


void AR60xSendPacket::jointSetPosition(uint8_t number, double value)
{
    double lowerLimit = desc_->joints[number].limits.lowerLimit;
    double upperLimit = desc_->joints[number].limits.upperLimit;
    double isReverse = desc_->joints[number].isReverse;
    uint8_t channel = desc_->joints[number].channel;

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
    short channel = desc_->joints[number].channel;
    write_int16(channel * 16 + JointOffsetAddress, angle_to_uint16(value));
}


void AR60xSendPacket::jointSetPIDGains(uint8_t number, JointData::PIDGains gains)
{
    short channel = desc_->joints[number].channel;
    write_int16(channel * 16 + JointPGainAddress, gains.proportional);
    write_int16(channel * 16 + JointIGaneAddress, gains.integral);
    write_int16(channel * 16 + JointDGainAddress, gains.derivative);
}

void AR60xSendPacket::jointSetLowerLimit(uint8_t number, double value)
{
    short channel = desc_->joints[number].channel;
    bool isReverce = desc_->joints[number].isReverse;

    if(isReverce)
        write_int16(channel * 16 + JointUpperLimitAddress, angle_to_uint16(-value));
    else
        write_int16(channel * 16 + JointLowerLimitAddress, angle_to_uint16(value));
}

void AR60xSendPacket::jointSetUpperLimit(uint8_t number, double value)
{
    short channel = desc_->joints[number].channel;
    bool isReverce = desc_->joints[number].isReverse;

    if(isReverce)
        write_int16(channel * 16 + JointLowerLimitAddress, angle_to_uint16(-value));
    else
        write_int16(channel * 16 + JointUpperLimitAddress, angle_to_uint16(value));
}

void AR60xSendPacket::jointSetState(uint8_t number, JointState::JointStates state)
{
    short channel = desc_->joints[number].channel;
    bool isEnable = desc_->joints[number].isEnable;

    switch (state) {
    case JointState::BRAKE:
        byte_array_[channel * 16 + 1] &= (255 - 3);
        break;
    case JointState::RELAX:
        byte_array_[channel * 16 + 1] &= (255 - 3);
        byte_array_[channel * 16 + 1] |= 2;
        break;
    case JointState::STOP:
        byte_array_[channel * 16 + 1] &= (255 - 3);
        byte_array_[channel * 16 + 1] |= 1;
        break;
    case JointState::TRACE:
        if(isEnable) byte_array_[channel * 16 + 1] |= 3;
        break;
    default:
        break;
    }
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


void AR60xSendPacket::write_int16(uint16_t address, int16_t value)
{
    byte_array_[address + 1] = (BYTE)(value >> 8);
    byte_array_[address] = value;
}

short AR60xSendPacket::angle_to_uint16(double angle)
{
    return (short)(angle * 100);
}