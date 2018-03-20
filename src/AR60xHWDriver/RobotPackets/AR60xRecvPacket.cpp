#include "AR60xRecvPacket.h"

AR60xRecvPacket::AR60xRecvPacket(AR60xDescription *robotDesc)
{
    desc = robotDesc;
}

void AR60xRecvPacket::initFromByteArray(const char bytes[])
{
    locker.lock();
    for(int i = 0; i < packetSize; i++)
        byteArray[i] = bytes[i];
    locker.unlock();
}

short AR60xRecvPacket::sensorGetValue(short number)
{
    locker.lock();
    int channel = desc->sensors.at(number).channel;
    int16_t value = readInt16(channel * 16 + sensorsMap.at(number));
    locker.unlock();
    return value;
}

int16_t AR60xRecvPacket::readInt16(uint16_t address)
{
    int16_t value = (byteArray[address + 1] << 8) + (BYTE)byteArray[address];
    return value;
}

float AR60xRecvPacket::readFloat(uint16_t address)
{
    float value = static_cast<float>((byteArray[address + 1] << 8) + (BYTE)byteArray[address]);
    return value;
}

short AR60xRecvPacket::jointGetCurrent(short number)
{
    locker.lock();
    int channel = desc->joints.at(number).channel;
    int16_t value = readInt16(channel * 16 + JointCurrentAddress);
    locker.unlock();
    return value;
}

short AR60xRecvPacket::jointGetVoltage(short number)
{
    locker.lock();
    int channel = desc->joints.at(number).channel;
    int16_t value = readInt16(channel * 16 + JointVoltageAddress);
    locker.unlock();
    return value;
}

short AR60xRecvPacket::jointGetPosition(short number)
{
    locker.lock();
    int channel = desc->joints.at(number).channel;
    int16_t value = readInt16(channel * 16 + JointPositionAddress);
    locker.unlock();
    return value;
}

short AR60xRecvPacket::jointGetPGain(short number)
{
    locker.lock();
    int channel = desc->joints.at(number).channel;
    int16_t value = readInt16(channel * 16 + JointPGainAddress);
    locker.unlock();
    return value;
}

short AR60xRecvPacket::jointGetIGain(short number)
{
    locker.lock();
    int channel = desc->joints.at(number).channel;
    int16_t value = readInt16(channel * 16 + JointIGaneAddress);
    locker.unlock();
    return value;
}

// TODO : возвращать не в short!!!
short AR60xRecvPacket::jointGetState(short number)
{
    locker.lock();
    int channel = desc->joints.at(number).channel;
    int16_t value = readInt16(channel * 16 + JointStateAddress);
    locker.unlock();
    return value;
}

short AR60xRecvPacket::jointGetLowerLimit(short number)
{
    int16_t value;
    locker.lock();
    int channel = desc->joints.at(number).channel;
    if(desc->joints.at(number).isReverse)
        value = -1 * readInt16(channel * 16 + JointUpperLimitAddress);
    else
        value = readInt16(channel * 16 + JointLowerLimitAddress);
    locker.unlock();
    return value;
}

short AR60xRecvPacket::jointGetUpperLimit(short number)
{
    int16_t value;
    locker.lock();
    int channel = desc->joints.at(number).channel;
    if(desc->joints.at(number).isReverse)
        value = -1 * readInt16(channel * 16 + JointLowerLimitAddress);
    else
        value = readInt16(channel * 16 + JointUpperLimitAddress);
    locker.unlock();
    return value;
}

float AR60xRecvPacket::supplyGetVoltage(PowerData::PowerSupplies supply)
{
   locker.lock();
   int address = powerStateMap.at(supply).SupplyVoltageAddress;
   float value = readFloat(address) / 1000;
   if(address == Supply48VoltageAddress) value *= 10;
   locker.unlock();
   return value;
}

float AR60xRecvPacket::supplyGetCurrent(PowerData::PowerSupplies supply)
{
    locker.lock();
    int address = powerStateMap.at(supply).SupplyCurrentAddress;
    float value = readFloat(address) / 1000;
    if(address == Supply48CurrentAddress) value *= 10;
    locker.unlock();
    return value;
}

const char *AR60xRecvPacket::getByteArray()
{
    return byteArray;
}

std::mutex *AR60xRecvPacket::getMutex()
{
    return &locker;
}




