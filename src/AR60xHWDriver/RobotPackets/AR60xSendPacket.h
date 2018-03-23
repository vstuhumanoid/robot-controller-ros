#ifndef AR60XSENDPACKET_H
#define AR60XSENDPACKET_H

#include "AR60xPacketsDefinitions.h"
#include "BasePacket.h"
#include <DataTypes/JointState.h>
#include <DataTypes/SensorImuState.h>
#include <DataTypes/SensorFeetState.h>
#include <RobotDescription/AR60xDescription.h>

#include <iostream>
#include <map>
#include <mutex>
#include <stdlib.h>
#include <cstring>

class AR60xSendPacket : public BasePacket
{
public:
    AR60xSendPacket(AR60xDescription& robotDesc);

    void jointSetPosition(uint8_t number, double value);
    void jointSetOffset(uint8_t number, double value);
    void jointSetPIDGains(uint8_t number, JointData::PIDGains gains);
    void jointSetLowerLimit(uint8_t number, double value);
    void jointSetUpperLimit(uint8_t number, double value);
    void jointSetState(uint8_t number, JointState state );
    void supplySetOn( PowerData::PowerSupplies supply );
    void supplySetOff( PowerData::PowerSupplies supply );
    void sensorSetOffset(uint8_t groupId, uint8_t number, double value);
    void sensorSetImuOffset(SensorImuState data);
    void sensorSetFeetOffset(SensorFeetState data);
private:

    void sensorSetFootOffset(SensorFeetState::FootData data, uint8_t groupId);
    void write_int16(uint16_t address, int16_t value);
    short angle_to_uint16(double angle);

};

#endif // AR60XSENDPACKET_H
