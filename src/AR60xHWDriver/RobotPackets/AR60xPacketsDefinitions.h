#ifndef AR60XPACKETDEFINITION_H
#define AR60XPACKETDEFINITION_H

#include <iostream>
#include <map>
#include <DataTypes/PowerSources.h>

typedef unsigned char BYTE;

const uint16_t packetSize  = 1472;
const uint16_t countChannels = 71;

///////////////////////////////////////// SENSORS //////////////////////////////////////////////////////////////////////

// sensor groups ids
const uint8_t ImuGroupId = 1;
const uint8_t LeftFootGroupId = 2;
const uint8_t RightFootGroupId = 3;

// bit offset in sensors' packet
const uint16_t SensorYawOffset = 2;
const uint16_t SensorPitchOffset = 4;
const uint16_t SensorRollOffset = 6;

const uint16_t SensorAccXOffset = 8;
const uint16_t SensorAccYOffset = 10;
const uint16_t SensorAccZOffset = 12;

const uint16_t SensorTxOffset = 2;
const uint16_t SensorTyOffset = 4;
const uint16_t SensorFzOffset = 6;

const uint16_t SensorUch0Offset = 8;
const uint16_t SensorUch1Offset = 10;
const uint16_t SensorUch2Offset = 12;
const uint16_t SensorUch3Offset = 14;

static std::map<int ,int> sensorsMap =
{
    {1, SensorYawOffset},
    {2, SensorPitchOffset},
    {3, SensorRollOffset},
    {4, SensorAccXOffset},
    {5, SensorAccYOffset},
    {6, SensorAccZOffset},
    {7, SensorUch0Offset},
    {8, SensorUch1Offset},
    {9, SensorUch2Offset},
    {10, SensorUch3Offset},
    {11, SensorUch0Offset},
    {12, SensorUch1Offset},
    {13, SensorUch2Offset},
    {14, SensorUch3Offset}
};

/////////////////////////////////////// JOINTS /////////////////////////////////////////////////////////////////////////

// Joint's properties offset in joint package
// Common (Send & Receive)
const uint8_t DeviceNumberAddress =    0;
const uint8_t JointStateAddress =      1;
const uint8_t JointPositionAddress =   2;
const uint8_t JointPGainAddress =      8;
const uint8_t JointIGainAddress =      10;
const uint8_t JointLowerLimitAddress = 12;
const uint8_t JointUpperLimitAddress = 14;

// Send packet
const uint8_t JointDGainAddress =      4;
const uint8_t JointOffsetAddress  =    6;

// Receive packet
const uint8_t JointVoltageAddress =    4;
const uint8_t JointCurrentAddress =    6;

/////////////////////////////////// POWER //////////////////////////////////////////////////////////////////////////////

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

static std::map<PowerSources, PowerStateAddress> powerStateMap =
{
    {PowerSources::Supply12V, {Supply12VoltageAddress, Supply12CurrentAddress}},
    {PowerSources::Supply6V1, {Supply6V1VoltageAddress, Supply6V1CurrentAddress}},
    {PowerSources::Supply6V2, {Supply6V2VoltageAddress, Supply6V2CurrentAddress}},
    {PowerSources::Supply8V1, {Supply8V1VoltageAddress, Supply8V1CurrentAddress}},
    {PowerSources::Supply8V2, {Supply8V2VoltageAddress, Supply8V2CurrentAddress}},
    {PowerSources::Supply48V, {Supply48VoltageAddress, Supply48CurrentAddress}}
};

#endif // AR60XPACKETDEFINITION_H
