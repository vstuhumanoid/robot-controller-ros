#ifndef AR60XSENDPACKET_H
#define AR60XSENDPACKET_H

#include "AR60xPacketsDefinitions.h"
#include "../DataTypes/JointState.h"
#include "../RobotDescription/AR60xDescription.h"

#include <iostream>
#include <map>
#include <mutex>
#include <stdlib.h>

class AR60xSendPacket
{
public:
    AR60xSendPacket(AR60xDescription *robotDesc);

    void initFromByteArray( unsigned char bytes[] );
    void init(void);
    const char *getByteArray();
    int getSize(){ return packetSize; }
    std::mutex *getMutex();

    void jointSetPosition( short number, short value );
    void jointSetOffset( short number, short value );
    void jointSetPGain( short number, short value );
    void jointSetIGain( short number, short value );
    void jointSetDGain( short number, short value );
    void jointSetLowerLimit( short number, short value );
    void jointSetUpperLimit( short number, short value );
    void jointSetState( short number , JointState::JointStates state );

    void supplySetOn( PowerData::PowerSupplies supply );
    void supplySetOff( PowerData::PowerSupplies supply );

private:
    AR60xDescription * desc;

    char byteArray [packetSize];
    std::mutex locker;

    void writeInt16(uint16_t address, int16_t value);
};

#endif // AR60XSENDPACKET_H
