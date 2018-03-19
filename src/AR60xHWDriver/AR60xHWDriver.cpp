#include "AR60xHWDriver.h"


AR60xHWDriver::AR60xHWDriver(size_t max_recv_packet_size) :
    sendpacket(nullptr),
    recvPacket(nullptr),
    connection(nullptr)
{
    max_recv_packet_size_ = max_recv_packet_size;
}

AR60xHWDriver::AR60xHWDriver(std::string config_filename, size_t max_recv_packet_size) : AR60xHWDriver(max_recv_packet_size)
{
    loadConfig(config_filename);
}

AR60xHWDriver::~AR60xHWDriver()
{
    if(sendpacket)
    {
        delete[] sendpacket;
        sendpacket = nullptr;
    }

    if(recvPacket)
    {
        delete[] recvPacket;
        recvPacket = nullptr;
    }

    if(connection)
    {
        delete[] connection;
        connection = nullptr;
    }
}

void AR60xHWDriver::loadConfig(std::string fileName)
{
    XMLSerializer serializer;
    if(!serializer.deserialize(fileName, &desc, &connectionData))
    {
        ROS_ERROR("Config parsing error");
        throw std::runtime_error("Config parsing error");
    }

    init_packets();
}

bool AR60xHWDriver::saveConfig(std::string fileName)
{
    XMLSerializer serializer;
    return serializer.serialize(fileName, &desc, &connectionData);
}

void AR60xHWDriver::init_packets()
{
    sendpacket = new AR60xSendPacket(&desc);
    recvPacket = new AR60xRecvPacket(&desc);

    // TODO: Max receive packet size
    connection = new UDPConnection(*sendpacket, *recvPacket, connectionData.sendDelay, max_recv_packet_size_);
}


// ------------------------------ connection --------------------------------------------


void AR60xHWDriver::robotConnect()
{
    connection->connectToHost(connectionData.host, connectionData.recvPort);
}

void AR60xHWDriver::robotDisconnect()
{
    connection->breakConnection();
}


// ------------------------------ joints ------------------------------------------------

// settings
void AR60xHWDriver::JointSetSettings(int joint, JointData settings)
{
    desc.joints[joint] = settings;
}

JointData AR60xHWDriver::JointGetSettings(int joint)
{
    return desc.joints[joint];
}

void AR60xHWDriver::JointSetSettings(std::vector<int> joints, std::vector<JointData> settings)
{
    if(joints.size() != settings.size())
        throw std::invalid_argument("Count of joints and settings must be equal");

    for(int i = 0; i < joints.size(); i++)
        desc.joints[i] = settings[i];
}

std::vector<JointData> AR60xHWDriver::JointGetSettings(std::vector<int> joints)
{
    std::vector<JointData> data(joints.size());
    for(int i = 0; i<joints.size(); i++)
        data[i] = desc.joints[i];
}


// position
void AR60xHWDriver::JointSetPosition(int joint, int position)
{
    sendpacket->jointSetPosition(joint, position);
}

int AR60xHWDriver::JointGetPosition(int joint)
{
    return recvPacket->jointGetPosition(joint);
}

void AR60xHWDriver::JointSetPosition(std::vector<int>& joints, std::vector<int>& position)
{
    if(joints.size() != position.size())
        throw std::invalid_argument("Count of joints and position must be equal");

    for(int i = 0; i < joints.size(); i++)
        sendpacket->jointSetPosition(joints[i], position[i]);
}

std::vector<int> AR60xHWDriver::JointGetPosition(std::vector<int>& joints)
{
    std::vector<int> positions(joints.size());
    for(int i = 0; i < joints.size(); i++)
        positions[i] = recvPacket->jointGetPosition(i);

    return positions;
}


// offset
void AR60xHWDriver::JointSetOffset(int joint, int offset)
{
    //TODO: Use only desc, package read from desc
    desc.joints[joint].offset = offset;
    sendpacket->jointSetOffset(joint, offset);
}


int AR60xHWDriver::JointGetOffset(int joint)
{
    return desc.joints[joint].offset;
}

void AR60xHWDriver::JointSetOffset(std::vector<int> joints, std::vector<int> offset)
{
    if(joints.size() != offset.size())
        throw std::invalid_argument("Count of joints and offset must be equal");

    for(int i = 0; i < joints.size(); i++)
        desc.joints[i].offset = offset[i];
}

std::vector<int> AR60xHWDriver::JointGetOffset(std::vector<int> joints)
{
    std::vector<int> offsets(joints.size());
    for(int i = 0; i < joints.size(); i++)
        offsets[i] = desc.joints[i].offset;
}


// reverse
void AR60xHWDriver::JointSetReverce(int joint, bool isReverce)
{
    desc.joints[joint].isReverce = isReverce;
}

bool AR60xHWDriver::JointGetReverce(int joint)
{
    return desc.joints[joint].isReverce;
}

void AR60xHWDriver::JointSetReverce(std::vector<int> joints, std::vector<int> isReverce)
{
    throw std::runtime_error("Not implemented");
}

std::vector<bool> AR60xHWDriver::JointGetReverce(std::vector<int> joints)
{
    throw std::runtime_error("Not implemented");
}


// PID
void AR60xHWDriver::JointSetPIDGains(int joint, JointData::PIDGains gains)
{
    //TODO: Add to config later
    sendpacket->jointSetPGain(joint, gains.proportional);
    sendpacket->jointSetIGain(joint, gains.integral);
    sendpacket->jointSetDGain(joint, gains.derivative);
}

JointData::PIDGains AR60xHWDriver::JointGetPIDGains(int joint)
{
    //TODO: Read real pids from robot?
    return desc.joints[joint].gains;
}

void AR60xHWDriver::JointSetPIDGains(std::vector<int> joints, std::vector<JointData::PIDGains> gains)
{
    throw std::runtime_error("Not implemented");
}

std::vector<JointData::PIDGains> AR60xHWDriver::JointGetPIDGains(std::vector<int> joints)
{
    throw std::runtime_error("Not implemented");
}


// limits
void AR60xHWDriver::JointSetLimits(int joint, JointData::JointLimits limits)
{
    desc.joints[joint].limits = limits;
    sendpacket->jointSetLowerLimit(joint, limits.lowerLimit);
    sendpacket->jointSetUpperLimit(joint, limits.upperLimit);
}

// TODO: Удалить JointSettings перенсти вместо него JointInformation!!!!!
JointData::JointLimits AR60xHWDriver::JointGetLimits(int joint)
{
    return desc.joints.at(joint).limits;
}

void AR60xHWDriver::JointSetLimits(std::vector<int> joints, std::vector<JointData::JointLimits> limits)
{
    throw std::runtime_error("Not implemented");
}

std::vector<JointData::JointLimits> AR60xHWDriver::JointGetLimits(std::vector<int> joints)
{
    throw std::runtime_error("Not implemented");
}


// enable
void AR60xHWDriver::JointSetEnable(int joint, bool isEnable)
{
    desc.joints.at(joint).isEnable = isEnable;
}

bool AR60xHWDriver::JointGetEnable(int joint)
{
    return desc.joints.at(joint).isEnable;
}

void AR60xHWDriver::JointSetEnable(std::vector<int> joints, std::vector<bool> isEnable)
{
    throw std::runtime_error("Not implemented");
}

std::vector<bool> AR60xHWDriver::JointGetEnable(std::vector<int> joints)
{
    throw std::runtime_error("Not implemented");
}

// state
void AR60xHWDriver::JointSetState(int joint, JointState::JointStates state)
{
    sendpacket->jointSetState(joint, state);
}

//TODO: проверить!!!!
JointState AR60xHWDriver::JointGetState(int joint)
{
    //return recvPacket->jointGetState(joint);
}

void AR60xHWDriver::JointSetState(std::vector<int> joints, std::vector<JointState::JointStates> state)
{
    throw std::runtime_error("Not implemented");
}

std::vector<JointState> AR60xHWDriver::JointGetState(std::vector<int> joints)
{
    throw std::runtime_error("Not implemented");
}

// ------------------------------ power control -----------------------------------------

PowerState::PowerSupplyState AR60xHWDriver::JointGetSupplyState(int joint)
{
    PowerState::PowerSupplyState state;
    state.Voltage = recvPacket->jointGetVoltage(joint);
    state.Current = recvPacket->jointGetCurrent(joint);
    return state;
}

PowerState::PowerSupplyState AR60xHWDriver::PowerGetSupplyState(PowerData::PowerSupplies supply)
{
    PowerState::PowerSupplyState state;
    state.Voltage = recvPacket->supplyGetVoltage(supply);
    state.Current = recvPacket->supplyGetCurrent(supply);
    return state;
}

void AR60xHWDriver::SupplySetOnOff(PowerData::PowerSupplies supply, bool onOffState)
{
    if(onOffState)
        sendpacket->supplySetOn(supply);
    else
        sendpacket->supplySetOff(supply);
}

bool AR60xHWDriver::SupplyGetOnOff(PowerData::PowerSupplies supply)
{
    //TODO: нет метода в recvpacket
}

// ------------------------------ sensors -----------------------------------------------

SensorState AR60xHWDriver::SensorGetState(int sensor)
{
    //TODO: Get sensors values
    //return recvPacket->sensorGetValue(sensor);
}



AR60xDescription *AR60xHWDriver::getRobotDesc()
{
    return &desc;
}

