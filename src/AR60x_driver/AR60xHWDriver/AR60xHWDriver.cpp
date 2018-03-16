#include "AR60xHWDriver.h"


AR60xHWDriver::AR60xHWDriver()
{
    desc = new AR60xDescription();
}

bool AR60xHWDriver::loadConfig(std::string fileName)
{
    XMLSerializer *serializer = new XMLSerializer();
    return serializer->deserialize(fileName, desc, &connectionData);
    //return deserialize(fileName);
}

void AR60xHWDriver::initPackets()
{
    sendpacket = new AR60xSendPacket(desc);
    recvPacket = new AR60xRecvPacket(desc);

    connection = new UDPConnection();

    connection->setRecvPacket(recvPacket);
    connection->setSendPacket(sendpacket);

    connection->initPackets();
}

void AR60xHWDriver::robotConnect()
{
    connection->connectToHost(connectionData.host,
                              connectionData.recvPort,
                              connectionData.sendDelay);
}

void AR60xHWDriver::robotDisconnect()
{
    connection->breakConnection();
}

void AR60xHWDriver::JointSetSettings(int joint, JointData settings)
{
    desc->joints.at(joint) = settings;
}

void AR60xHWDriver::JointSetPosition(int joint, int position)
{
    sendpacket->jointSetPosition(joint, position);
}

void AR60xHWDriver::JointSetOffset(int joint, int offset)
{
    desc->joints.at(joint).offset = offset;
    sendpacket->jointSetOffset(joint, offset);
}

void AR60xHWDriver::JointSetReverce(int joint, bool isReverce)
{
    desc->joints.at(joint).isReverce = isReverce;
}

void AR60xHWDriver::JointSetPIDGains(int joint, JointData::PIDGains gains)
{
    sendpacket->jointSetPGain(joint, gains.proportional);
    sendpacket->jointSetIGain(joint, gains.integral);
    sendpacket->jointSetDGain(joint, gains.derivative);
}

void AR60xHWDriver::JointSetLimits(int joint, JointData::JointLimits limits)
{
    sendpacket->jointSetLowerLimit(joint, limits.lowerLimit);
    sendpacket->jointSetUpperLimit(joint, limits.upperLimit);
}

void AR60xHWDriver::JointSetEnable(int joint, bool isEnable)
{
    desc->joints.at(joint).isEnable = isEnable;
}

void AR60xHWDriver::JointSetState(int joint, JointState::JointStates state)
{
    sendpacket->jointSetState(joint, state);
}

JointData AR60xHWDriver::JointGetSettings(int joint)
{
    return desc->joints.at(joint);
}

int AR60xHWDriver::JointGetPosition(int joint)
{
    return recvPacket->jointGetPosition(joint);
}

//TODO: проверить!!!!
JointState AR60xHWDriver::JointGetState(int joint)
{
    //return recvPacket->jointGetState(joint);
}

bool AR60xHWDriver::JointGetReverce(int joint)
{
    return desc->joints.at(joint).isReverce;
}

PowerState::PowerSupplyState AR60xHWDriver::JointGetSupplyState(int joint)
{
    PowerState::PowerSupplyState state;
    state.Voltage = recvPacket->jointGetVoltage(joint);
    state.Current = recvPacket->jointGetCurrent(joint);
    return state;
}

//TODO: Удалить JointSettings перенсти вместо него JointInformation!!!!!
JointData::JointLimits AR60xHWDriver::JointGetLimits(int joint)
{
    return desc->joints.at(joint).limits;
}

bool AR60xHWDriver::JointGetEnable(int joint)
{
    return desc->joints.at(joint).isEnable;
}

JointData::PIDGains AR60xHWDriver::JointGetPIDGains(int joint)
{
    return desc->joints.at(joint).gains;
}

void AR60xHWDriver::PowerSetSettings(PowerData settings)
{

}

void AR60xHWDriver::SupplySetState(PowerData::PowerSupplies supply, bool onOffState)
{
    if(onOffState)
        sendpacket->supplySetOn(supply);
    else
        sendpacket->supplySetOff(supply);
}

bool AR60xHWDriver::PowerGetOnOff(PowerData::PowerSupplies supply)
{
    //TODO: нет метода в recvpacket
}

PowerState::PowerSupplyState AR60xHWDriver::PowerGetSupplyState(PowerData::PowerSupplies supply)
{
    PowerState::PowerSupplyState state;
    state.Voltage = recvPacket->supplyGetVoltage(supply);
    state.Current = recvPacket->supplyGetCurrent(supply);
    return state;
}

SensorState AR60xHWDriver::SensorGetState(int sensor)
{
    //return recvPacket->sensorGetValue(sensor);
}

AR60xDescription *AR60xHWDriver::getRobotDesc()
{
    return desc;
}

bool AR60xHWDriver::saveConfig(std::string fileName)
{
//    JointData joint;
//    joint.channel = 2;
//    joint.isEnable = false;
//    joint.isReverce = false;
//    joint.name = "joint 1";
//    joint.offset = 34455;
//    joint.number = 1;

//    JointData::PIDGains gates;
//    gates.derivative = 5;
//    gates.integral = 6;
//    gates.proportional = 1200;

//    joint.gains = gates;

//    JointData::JointLimits limits;
//    limits.lowerLimit = -1300;
//    limits.upperLimit = 1500;

//    joint.limits = limits;

//    desc->joints[1] = joint;

//    JointData joint2;

//    joint2.number = 2;

//    joint2.channel = 2;
//    joint2.isEnable = false;
//    joint2.isReverce = false;
//    joint2.name = "joint 2";
//    joint2.offset = 34455;

//    gates.derivative = 5;
//    gates.integral = 6;
//    gates.proportional = 1200;

//    joint2.gains = gates;

//    limits.lowerLimit = -1300;
//    limits.upperLimit = 1500;

//    joint2.limits = limits;

//    desc->joints[2] = joint2;

//    SensorData sensor1;
//    sensor1.number = 1;
//    sensor1.name = "Датчик скорости";
//    sensor1.channel = 1;

//    SensorData sensor2;
//    sensor2.number = 2;
//    sensor2.name = "Датчик давления";
//    sensor2.channel = 2;

//    desc->sensors[1] = sensor1;
//    desc->sensors[2] = sensor2;

    XMLser = new XMLSerializer();
    return XMLser->serialize(fileName, desc, &connectionData);
    //return serialize(fileName);
}
