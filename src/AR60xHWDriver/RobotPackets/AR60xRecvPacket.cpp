#include "AR60xRecvPacket.h"

AR60xRecvPacket::AR60xRecvPacket(AR60xDescription& robotDesc) : BasePacket(robotDesc)
{
}

void AR60xRecvPacket::initFromByteArray(const char bytes[])
{
    for(int i = 0; i < packetSize; i++)
        byte_array_[i] = bytes[i];
}


PowerState::PowerSupplyState AR60xRecvPacket::jointGetSupplyState(uint8_t number)
{
    PowerState::PowerSupplyState state;
    int channel = desc_.joints[number].channel;
    state.Current = read_int16(channel * 16 + JointCurrentAddress) / 1000.0f;
    state.Voltage = read_int16(channel * 16 + JointVoltageAddress) / 1000.0f;
    return state;
}

double AR60xRecvPacket::jointGetPosition(uint8_t number)
{
    int channel = desc_.joints.at(number).channel;
    int16_t value = read_int16(channel * 16 + JointPositionAddress);
    return value;
}

JointData::PIDGains AR60xRecvPacket::jointGetPIDGains(uint8_t number)
{
    // Can read from robot only P and I gains, but D can be read only
    // from desc.
    // Fuck AT, just return from description
    return desc_.joints[number].gains;
}

JointState AR60xRecvPacket::jointGetState(short number)
{
    JointState state;

    int channel = desc_.joints.at(number).channel;
    uint8_t value = byte_array_[channel * 16 + JointStateAddress];
    uint8_t status_mask = value & 0b11;

    if(status_mask == 0b00)
        state.state = JointState::BRAKE;
    else if(status_mask == 0b01)
        state.state = JointState::STOP;
    else if(status_mask == 0b10)
        state.state = JointState::RELAX;
    else if(status_mask == 0b11)
        state.state = JointState::TRACE;

    state.isBeyondLowerLimit = value & (1 << 4);
    state.isBeyondUpperLimit = value & (1 << 5);
    state.controlType = (value & (1<<7)) ? JointState::POSITION_CONTROl : JointState::TORQUE_CONTROL;

    return state;
}

double AR60xRecvPacket::jointGetLowerLimit(uint8_t number)
{
    int16_t value;
    
    int channel = desc_.joints.at(number).channel;
    if(desc_.joints.at(number).isReverse)
        value = -read_int16(channel * 16 + JointUpperLimitAddress);
    else
        value = read_int16(channel * 16 + JointLowerLimitAddress);
    
    return  uint16_to_angle(value);
}

double AR60xRecvPacket::jointGetUpperLimit(uint8_t number)
{
    int16_t value;
    
    int channel = desc_.joints.at(number).channel;
    if(desc_.joints.at(number).isReverse)
        value = -read_int16(channel * 16 + JointLowerLimitAddress);
    else
        value = read_int16(channel * 16 + JointUpperLimitAddress);
    
    return uint16_to_angle(value);
}

PowerState::PowerSupplyState AR60xRecvPacket::supplyGetState(PowerData::PowerSupplies supply)
{
    PowerState::PowerSupplyState state;
    state.Voltage = supplyGetVoltage(supply);
    state.Current = supplyGetCurrent(supply);
    return state;
}

float AR60xRecvPacket::supplyGetVoltage(PowerData::PowerSupplies supply)
{
   
   int address = powerStateMap.at(supply).SupplyVoltageAddress;
   float value = read_float(address) / 1000;
   if(address == Supply48VoltageAddress) value *= 10;
   
   return value;
}

float AR60xRecvPacket::supplyGetCurrent(PowerData::PowerSupplies supply)
{
    
    int address = powerStateMap.at(supply).SupplyCurrentAddress;
    float value = read_float(address) / 1000;
    if(address == Supply48CurrentAddress) value *= 10;
    
    return value;
}

double AR60xRecvPacket::sensorGetValue(short number)
{
    int channel = desc_.sensors.at(number).channel;
    int16_t value = read_int16(channel * 16 + sensorsMap.at(number));
    return value / 100;
}

ImuData AR60xRecvPacket::sensorGetImu()
{

}

LegsData AR60xRecvPacket::sensorGetLegs()
{

}


int16_t AR60xRecvPacket::read_int16(uint16_t address)
{
    int16_t value = (byte_array_[address + 1] << 8) + (BYTE)byte_array_[address];
    return value;
}

float AR60xRecvPacket::read_float(uint16_t address)
{
    float value = static_cast<float>((byte_array_[address + 1] << 8) + (BYTE)byte_array_[address]);
    return value;
}

double AR60xRecvPacket::uint16_to_angle(uint16_t angle)
{
    return angle / 100.0;
}


