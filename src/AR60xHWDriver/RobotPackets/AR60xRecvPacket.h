#ifndef AR60XRECVPACKET_H
#define AR60XRECVPACKET_H

#include <iostream>
#include <map>
#include <mutex>
#include <cstdint>

#include "BasePacket.h"
#include "AR60xPacketsDefinitions.h"
#include <RobotDescription/AR60xDescription.h>

class AR60xRecvPacket : public BasePacket
{
public:
    AR60xRecvPacket(AR60xDescription *robotDesc);

    double jointGetPosition(uint8_t number);
    JointData::PIDGains jointGetPIDGains(uint8_t number);

    double jointGetLowerLimit(uint8_t number);
    double jointGetUpperLimit(uint8_t number);

    short jointGetCurrent(uint8_t number);
    short jointGetVoltage(uint8_t number);
    float supplyGetVoltage(PowerData::PowerSupplies supply);
    float supplyGetCurrent(PowerData::PowerSupplies supply);

    short sensorGetValue( short number );

private:

    int16_t read_int16(uint16_t address);
    float read_float(uint16_t address);
    double uint16_to_angle(uint16_t angle);
};

#endif // AR60XRECVPACKET_H
