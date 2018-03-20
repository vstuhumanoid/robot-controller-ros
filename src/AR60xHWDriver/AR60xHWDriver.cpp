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
void AR60xHWDriver::JointSetSettings(uint8_t joint, JointData settings)
{
    desc.joints[joint] = settings;
}

JointData AR60xHWDriver::JointGetSettings(uint8_t joint)
{
    return desc.joints[joint];
}

void AR60xHWDriver::JointSetSettings(std::vector<uint8_t> joints, std::vector<JointData> settings)
{
    if(joints.size() != settings.size())
        throw std::invalid_argument("Count of joints and settings must be equal");

    for(int i = 0; i < joints.size(); i++)
        desc.joints[joints[i]] = settings[i];
}

std::vector<JointData> AR60xHWDriver::JointGetSettings(std::vector<uint8_t> joints)
{
    std::vector<JointData> data(joints.size());
    for(int i = 0; i<joints.size(); i++)
        data[i] = desc.joints[joints[i]];
}


// position
void AR60xHWDriver::JointSetPosition(uint8_t joint, double position)
{
    sendpacket->jointSetPosition(joint, position);
}

double AR60xHWDriver::JointGetPosition(uint8_t joint)
{
    return recvPacket->jointGetPosition(joint);
}

void AR60xHWDriver::JointSetPosition(std::vector<uint8_t>& joints, std::vector<double>& position)
{
    if(joints.size() != position.size())
        throw std::invalid_argument("Count of joints and position must be equal");

    for(int i = 0; i < joints.size(); i++)
        sendpacket->jointSetPosition(joints[i], position[i]);
}

std::vector<double> AR60xHWDriver::JointGetPosition(std::vector<uint8_t>& joints)
{
    std::vector<double> positions(joints.size());
    for(int i = 0; i < joints.size(); i++)
        positions[i] = recvPacket->jointGetPosition(joints[i]);

    return positions;
}


// offset
void AR60xHWDriver::JointSetOffset(uint8_t joint, double offset)
{
    desc.joints[joint].offset = offset;
    sendpacket->jointSetOffset(joint, offset);
}


double AR60xHWDriver::JointGetOffset(uint8_t joint)
{
    return desc.joints[joint].offset;
}

void AR60xHWDriver::JointSetOffset(std::vector<uint8_t> joints, std::vector<double> offset)
{
    if(joints.size() != offset.size())
        throw std::invalid_argument("Count of joints and offset must be equal");

    for(int i = 0; i < joints.size(); i++)
    {
        sendpacket->jointSetOffset(joints[i], offset[i]);
        desc.joints[joints[i]].offset = offset[i];
    }
}

std::vector<double> AR60xHWDriver::JointGetOffset(std::vector<uint8_t> joints)
{
    std::vector<double> offsets(joints.size());
    for(int i = 0; i < joints.size(); i++)
        offsets[i] = desc.joints[joints[i]].offset;

    return offsets;
}


// reverse
void AR60xHWDriver::JointSetReverce(uint8_t joint, bool isReverce)
{
    desc.joints[joint].isReverse = isReverce;
}

bool AR60xHWDriver::JointGetReverce(uint8_t joint)
{
    return desc.joints[joint].isReverse;
}

void AR60xHWDriver::JointSetReverce(std::vector<uint8_t> joints, std::vector<bool> isReverse)
{
    if(joints.size() != isReverse.size())
        throw std::invalid_argument("Count of joints and isReverse must be equal");

    for(int i = 0; i < joints.size(); i++)
        desc.joints[joints[i]].isReverse = isReverse[i];

}

std::vector<bool> AR60xHWDriver::JointGetReverce(std::vector<uint8_t> joints)
{
    std::vector<bool> isReverse(joints.size());
    for(int i = 0; i<joints.size(); i++)
        isReverse[i] = desc.joints[joints[i]].isReverse;

    return isReverse;
}


// PID
void AR60xHWDriver::JointSetPIDGains(uint8_t joint, JointData::PIDGains gains)
{
    //TODO: Write to desc
    sendpacket->jointSetPIDGains(joint, gains);
}

JointData::PIDGains AR60xHWDriver::JointGetPIDGains(uint8_t joint)
{
    //TODO: Read real pids from robot?
    return desc.joints[joint].gains;
}

void AR60xHWDriver::JointSetPIDGains(std::vector<uint8_t> joints, std::vector<JointData::PIDGains> gains)
{
    if (joints.size() != gains.size())
        throw std::invalid_argument("Count of joints and gains must be equal");

    for (int i = 0; i < joints.size(); i++)
    {
        //TODO: Write to desc
        sendpacket->jointSetPIDGains(joints[i], gains[i]);
    }
}

std::vector<JointData::PIDGains> AR60xHWDriver::JointGetPIDGains(std::vector<uint8_t> joints)
{
    std::vector<JointData::PIDGains> pids(joints.size());
    for(int i = 0; i<joints.size(); i++)
        pids[i] = desc.joints[joints[i]].gains;

    return pids;
}


// limits
void AR60xHWDriver::JointSetLimits(uint8_t joint, JointData::JointLimits limits)
{
    desc.joints[joint].limits = limits;
    sendpacket->jointSetLowerLimit(joint, limits.lowerLimit);
    sendpacket->jointSetUpperLimit(joint, limits.upperLimit);
}

// TODO: Удалить JointSettings перенсти вместо него JointInformation!!!!!
JointData::JointLimits AR60xHWDriver::JointGetLimits(uint8_t joint)
{
    return desc.joints[joint].limits;
}

void AR60xHWDriver::JointSetLimits(std::vector<uint8_t> joints, std::vector<JointData::JointLimits> limits)
{
    if(joints.size() != limits.size())
        throw std::invalid_argument("Count of joints and limits must be equal");

    for(int i = 0; i<joints.size(); i++)
    {
        desc.joints[joints[i]].limits = limits[i];
        sendpacket->jointSetLowerLimit(joints[i], limits[i].lowerLimit);
        sendpacket->jointSetUpperLimit(joints[i], limits[i].upperLimit);
    }
}

std::vector<JointData::JointLimits> AR60xHWDriver::JointGetLimits(std::vector<uint8_t> joints)
{
    std::vector<JointData::JointLimits> limits(joints.size());
    for(int i = 0; i<joints.size(); i++)
        limits[i] = desc.joints[joints[i]].limits;

    return limits;
}


// enable
void AR60xHWDriver::JointSetEnable(uint8_t joint, bool isEnable)
{
    desc.joints[joint].isEnable = isEnable;
}

bool AR60xHWDriver::JointGetEnable(uint8_t joint)
{
    return desc.joints[joint].isEnable;
}

void AR60xHWDriver::JointSetEnable(std::vector<uint8_t> joints, std::vector<bool> isEnable)
{
    if(joints.size() != isEnable.size())
        throw std::invalid_argument("Count of joints and isEnable must be equal");

    for(int i = 0; i<joints.size(); i++)
        desc.joints[joints[i]].isEnable = isEnable[i];
}

std::vector<bool> AR60xHWDriver::JointGetEnable(std::vector<uint8_t> joints)
{
    std::vector<bool> isEnable(joints.size());
    for(int i = 0; i < joints.size(); i++)
        isEnable[i] = desc.joints[joints[i]].isEnable;

    return isEnable;
}

// state
void AR60xHWDriver::JointSetState(uint8_t joint, JointState::JointStates state)
{
    sendpacket->jointSetState(joint, state);
}

//TODO: проверить!!!!
JointState AR60xHWDriver::JointGetState(uint8_t joint)
{
    //return recvPacket->jointGetState(joint);
    throw std::runtime_error("Not implemented");
}

void AR60xHWDriver::JointSetState(std::vector<uint8_t> joints, std::vector<JointState::JointStates> state)
{
    if(joints.size() != state.size())
        throw std::invalid_argument("Count of joints and state must be equal");

    for(int i = 0; i<joints.size(); i++)
        sendpacket->jointSetState(joints[i], state[i]);
}

std::vector<JointState> AR60xHWDriver::JointGetState(std::vector<uint8_t> joints)
{
    std::vector<JointState> states(joints.size());
    for(int  i = 0; i<joints.size(); i++)
    {
        //TODO: Read joint state
        throw std::runtime_error("Not implemented");
    }

    return states;
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

