#ifndef AR60XRECVPACKET_H
#define AR60XRECVPACKET_H

#include <iostream>
#include <map>
#include <mutex>
#include <cstdint>

#include "BasePacket.h"
#include "AR60xPacketsDefinitions.h"
#include <RobotDescription/AR60xDescription.h>
#include <DataTypes/JointState.h>
#include <DataTypes/ImuData.h>
#include <DataTypes/LegsData.h>

class AR60xRecvPacket : public BasePacket
{
public:
    AR60xRecvPacket(AR60xDescription& robotDesc);
    void initFromByteArray(const char bytes[]);

    double jointGetPosition(uint8_t number);
    JointData::PIDGains jointGetPIDGains(uint8_t number);
    JointState jointGetState(short number);
    double jointGetLowerLimit(uint8_t number);
    double jointGetUpperLimit(uint8_t number);
    PowerState::PowerSupplyState jointGetSupplyState(uint8_t number);
    PowerState::PowerSupplyState supplyGetState(PowerData::PowerSupplies supply);

    double sensorGetValue( short number );
    ImuData sensorGetImu();
    LegsData sensorGetLegs();

private:
    float supplyGetVoltage(PowerData::PowerSupplies supply);
    float supplyGetCurrent(PowerData::PowerSupplies supply);

    int16_t read_int16(uint16_t address);
    float read_float(uint16_t address);
    double uint16_to_angle(uint16_t angle);
};

#endif // AR60XRECVPACKET_H
