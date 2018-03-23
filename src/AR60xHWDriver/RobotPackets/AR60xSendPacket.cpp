#include "AR60xSendPacket.h"

AR60xSendPacket::AR60xSendPacket(AR60xDescription& robotDesc) : BasePacket(robotDesc)
{
}


void AR60xSendPacket::jointSetPosition(uint8_t number, double value)
{
    double lowerLimit = desc_.joints[number].limits.lowerLimit;
    double upperLimit = desc_.joints[number].limits.upperLimit;
    double isReverse = desc_.joints[number].isReverse;
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


void AR60xSendPacket::write_int16(uint16_t address, int16_t value)
{
    byte_array_[address + 1] = (BYTE)(value >> 8);
    byte_array_[address] = value;
}

short AR60xSendPacket::angle_to_uint16(double angle)
{
    return (short)(angle * 100);
}