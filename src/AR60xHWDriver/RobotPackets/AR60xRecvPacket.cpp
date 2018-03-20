#include "AR60xRecvPacket.h"

AR60xRecvPacket::AR60xRecvPacket(AR60xDescription *robotDesc)
{
    desc = robotDesc;
}

void AR60xRecvPacket::initFromByteArray(const char bytes[])
{
    locker.lock();
    for(int i = 0; i < packetSize; i++)
        byte_array_[i] = bytes[i];
    locker.unlock();
}

short AR60xRecvPacket::sensorGetValue(short number)
{
    locker.lock();
    int channel = desc->sensors.at(number).channel;
    int16_t value = read_int16(channel * 16 + sensorsMap.at(number));
    locker.unlock();
    return value;
}


short AR60xRecvPacket::jointGetCurrent(uint8_t number)
{
    locker.lock();
    int channel = desc->joints.at(number).channel;
    int16_t value = read_int16(channel * 16 + JointCurrentAddress);
    locker.unlock();
    return value;
}

short AR60xRecvPacket::jointGetVoltage(uint8_t number)
{
    locker.lock();
    int channel = desc->joints.at(number).channel;
    int16_t value = read_int16(channel * 16 + JointVoltageAddress);
    locker.unlock();
    return value;
}

double AR60xRecvPacket::jointGetPosition(uint8_t number)
{
    locker.lock();
    int channel = desc->joints.at(number).channel;
    int16_t value = read_int16(channel * 16 + JointPositionAddress);
    locker.unlock();
    return value;
}

short AR60xRecvPacket::jointGetPGain(short number)
{
    locker.lock();
    int channel = desc->joints.at(number).channel;
    int16_t value = read_int16(channel * 16 + JointPGainAddress);
    locker.unlock();
    return value;
}

short AR60xRecvPacket::jointGetIGain(short number)
{
    locker.lock();
    int channel = desc->joints.at(number).channel;
    int16_t value = read_int16(channel * 16 + JointIGaneAddress);
    locker.unlock();
    return value;
}

// TODO : возвращать не в short!!!
short AR60xRecvPacket::jointGetState(short number)
{
    locker.lock();
    int channel = desc->joints.at(number).channel;
    int16_t value = read_int16(channel * 16 + JointStateAddress);
    locker.unlock();
    return value;
}

double AR60xRecvPacket::jointGetLowerLimit(uint8_t number)
{
    int16_t value;
    locker.lock();
    int channel = desc->joints.at(number).channel;
    if(desc->joints.at(number).isReverse)
        value = -1 * read_int16(channel * 16 + JointUpperLimitAddress);
    else
        value = read_int16(channel * 16 + JointLowerLimitAddress);
    locker.unlock();
    return value;
}

double AR60xRecvPacket::jointGetUpperLimit(uint8_t number)
{
    int16_t value;
    locker.lock();
    int channel = desc->joints.at(number).channel;
    if(desc->joints.at(number).isReverse)
        value = -1 * read_int16(channel * 16 + JointLowerLimitAddress);
    else
        value = read_int16(channel * 16 + JointUpperLimitAddress);
    locker.unlock();
    return value;
}

float AR60xRecvPacket::supplyGetVoltage(PowerData::PowerSupplies supply)
{
   locker.lock();
   int address = powerStateMap.at(supply).SupplyVoltageAddress;
   float value = read_float(address) / 1000;
   if(address == Supply48VoltageAddress) value *= 10;
   locker.unlock();
   return value;
}

float AR60xRecvPacket::supplyGetCurrent(PowerData::PowerSupplies supply)
{
    locker.lock();
    int address = powerStateMap.at(supply).SupplyCurrentAddress;
    float value = read_float(address) / 1000;
    if(address == Supply48CurrentAddress) value *= 10;
    locker.unlock();
    return value;
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


