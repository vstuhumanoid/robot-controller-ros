#ifndef ROBOT_CONTROLLER_ROS_BASEPACKET_H
#define ROBOT_CONTROLLER_ROS_BASEPACKET_H

#include <cstring>
#include <cstdint>
#include <RobotDescription/AR60xDescription.h>
#include "AR60xPacketsDefinitions.h"

class BasePacket
{
public:
    const char *getByteArray() { return byte_array_; }
    int getSize(){ return packetSize; }

protected:
    AR60xDescription * desc_;
    char byte_array_ [packetSize];
};


#endif //ROBOT_CONTROLLER_ROS_BASEPACKET_H
