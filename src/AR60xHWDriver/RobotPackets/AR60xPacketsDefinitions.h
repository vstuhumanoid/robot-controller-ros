#ifndef AR60XPACKETDEFINITION_H
#define AR60XPACKETDEFINITION_H

#include "../DataTypes/PowerData.h"

#include <iostream>
#include <map>

typedef unsigned char BYTE;

const uint16_t packetSize  = 1472;
const uint16_t countChannels = 71;

const uint16_t SensorYawAddress = 2;
const uint16_t SensorPitchAddress = 4;
const uint16_t SensorRollAddress = 6;

const uint16_t SensorUch0Address = 8;
const uint16_t SensorUch1Address = 10;
const uint16_t SensorUch2Address = 12;
const uint16_t SensorUch3Address = 14;

const uint16_t SensorTxAddress = 8;
const uint16_t SensorTyAddress = 10;
const uint16_t SensorFzAddress = 12;

static std::map<int ,int> sensorsMap =
{
    {1, SensorYawAddress},
    {2, SensorPitchAddress},
    {3, SensorRollAddress},
    {4, SensorTxAddress},
    {5, SensorTyAddress},
    {6, SensorFzAddress},
    {7, SensorUch0Address},
    {8, SensorUch1Address},
    {9, SensorUch2Address},
    {10, SensorUch3Address},
    {11, SensorUch0Address},
    {12, SensorUch1Address},
    {13, SensorUch2Address},
    {14, SensorUch3Address}
};

// Joint's properties shit in joint package
const uint8_t DeviceNumberAddress =    0;
const uint8_t JointStateAddress =      1;
const uint8_t JointPositionAddress =   2;
const uint8_t JointVoltageAddress =    4;
const uint8_t JointCurrentAddress =    6;
const uint8_t JointOffsetAddress  =    6;
const uint8_t JointPGainAddress =      8;
const uint8_t JointIGaneAddress =      10;
const uint8_t JointDGainAddress =      4;
const uint8_t JointLowerLimitAddress = 12;
const uint8_t JointUpperLimitAddress = 14;

const uint16_t PowerDataAddress = 1408;

const uint16_t Supply6V1VoltageAddress =   0 * 2 + PowerDataAddress;
const uint16_t Supply6V2VoltageAddress =   1 * 2 + PowerDataAddress;
const uint16_t Supply8V1VoltageAddress =   2 * 2 + PowerDataAddress;
const uint16_t Supply8V2VoltageAddress =   3 * 2 + PowerDataAddress;
const uint16_t Supply12VoltageAddress =    4 * 2 + PowerDataAddress;
const uint16_t Supply48VoltageAddress =    5 * 2 + PowerDataAddress;

const uint16_t Supply6V1CurrentAddress =   0 * 2 + 12 + PowerDataAddress;
const uint16_t Supply6V2CurrentAddress =   1 * 2 + 12 + PowerDataAddress;
const uint16_t Supply8V1CurrentAddress =   2 * 2 + 12 + PowerDataAddress;
const uint16_t Supply8V2CurrentAddress =   3 * 2 + 12 + PowerDataAddress;
const uint16_t Supply12CurrentAddress =    4 * 2 + 12 + PowerDataAddress;
const uint16_t Supply48CurrentAddress =    5 * 2 + 12 + PowerDataAddress;

struct PowerStateAddress
{
    int SupplyVoltageAddress;
    int SupplyCurrentAddress;
};

static std::map<int ,PowerStateAddress> powerStateMap =
{
    {PowerData::Supply12V, {Supply12VoltageAddress, Supply12CurrentAddress}},
    {PowerData::Supply6V1, {Supply6V1VoltageAddress, Supply6V1CurrentAddress}},
    {PowerData::Supply6V2, {Supply6V2VoltageAddress, Supply6V2CurrentAddress}},
    {PowerData::Supply8V1, {Supply8V1VoltageAddress, Supply8V1CurrentAddress}},
    {PowerData::Supply8V2, {Supply8V2VoltageAddress, Supply8V2CurrentAddress}},
    {PowerData::Supply48V, {Supply48VoltageAddress, Supply48CurrentAddress}}
};

#endif // AR60XPACKETDEFINITION_H
