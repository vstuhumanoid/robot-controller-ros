#ifndef AR60XRECVPACKET_H
#define AR60XRECVPACKET_H

#include <iostream>
#include <map>
#include <mutex>
#include <stdlib.h>

#include "AR60xPacketsDefinitions.h"
#include "../RobotDescription/AR60xDescription.h"

class AR60xRecvPacket
{
public:
    AR60xRecvPacket(AR60xDescription *robotDesc);

    void initFromByteArray( const char bytes[] );

    short sensorGetValue( short number );

    short jointGetCurrent( short number );
    short jointGetVoltage( short number );
    short jointGetPosition( short number );
    short jointGetPGain( short number );
    short jointGetIGain( short number );
    short jointGetState( short number );
    short jointGetLowerLimit( short number );
    short jointGetUpperLimit( short number );

    float supplyGetVoltage(PowerData::PowerSupplies supply);
    float supplyGetCurrent(PowerData::PowerSupplies supply);

    const char *getByteArray();
    std::mutex *getMutex();

private:
    AR60xDescription * desc;

    char byteArray [packetSize];
    std::mutex locker;

    int16_t readInt16(uint16_t address);
    float readFloat(uint16_t address);
};

#endif // AR60XRECVPACKET_H
