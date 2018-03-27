#include "AR60xHWDriver.h"


AR60xHWDriver::AR60xHWDriver():
    sendpacket_(nullptr),
    recv_packet_(nullptr),
    connection_(nullptr),
    send_mutex_(),
    recv_mutex_()
{
}

AR60xHWDriver::AR60xHWDriver(std::string config_filename) : AR60xHWDriver()
{
    LoadConfig(config_filename);
}

AR60xHWDriver::~AR60xHWDriver()
{
    if(connection_)
    {
        connection_->breakConnection();
        delete[] connection_;
        connection_ = nullptr;
    }

    if(sendpacket_)
    {
        delete[] sendpacket_;
        sendpacket_ = nullptr;
    }

    if(recv_packet_)
    {
        delete[] recv_packet_;
        recv_packet_ = nullptr;
    }
}

void AR60xHWDriver::LoadConfig(std::string fileName)
{
    XMLSerializer serializer;
    if(!serializer.deserialize(fileName, &desc, &connectionData))
    {
        ROS_ERROR("Config parsing error");
        throw std::runtime_error("Config parsing error");
    }

    init_packets();
}

bool AR60xHWDriver::SaveConfig(std::string fileName)
{
    XMLSerializer serializer;
    return serializer.serialize(fileName, &desc, &connectionData);
}

void AR60xHWDriver::init_packets()
{
    sendpacket_ = new AR60xSendPacket(desc);
    recv_packet_ = new AR60xRecvPacket(desc);
    recv_packet_->initFromByteArray(sendpacket_->getByteArray());
    connection_ = new UDPConnection(*sendpacket_, *recv_packet_,
                                    recv_mutex_, send_mutex_,
                                    connectionData.localPort,
                                    connectionData.sendDelay);
}


AR60xDescription& AR60xHWDriver::GetRobotDesc()
{
    return desc;
}


// ------------------------------ connection --------------------------------------------


void AR60xHWDriver::RobotConnect()
{
    connection_->connectToHost(connectionData.host, connectionData.robotPort);
}

void AR60xHWDriver::RobotDisconnect()
{
    connection_->breakConnection();
}


// ------------------------------ joints ------------------------------------------------

void AR60xHWDriver::SetStartPose()
{
    SEND_GUARD;
    JointState state;
    state.state = JointState::MotorState::TRACE;
    state.controlType = JointState::ControlType::POSITION_CONTROl;

    for(auto& joint: desc.joints)
    {
        //JointSetPosition(joint.second.number, 0);
        //JointSetState(joint.second.number, JointState::MotorState::TRACE);
        sendpacket_->jointSetPosition(joint.second.number, 0);
        sendpacket_->jointSetState(joint.second.number, state);
    }
}

// settings
void AR60xHWDriver::JointSetSettings(uint8_t joint, JointData settings)
{
    SEND_GUARD;
    desc.joints[joint] = settings;
}

JointData AR60xHWDriver::JointGetSettings(uint8_t joint)
{
    RECV_GUARD;
    return desc.joints[joint];
}

void AR60xHWDriver::JointSetSettings(std::vector<uint8_t> joints, std::vector<JointData> settings)
{
    SEND_GUARD;
    if(joints.size() != settings.size())
        throw std::invalid_argument("Count of joints and settings must be equal");

    for(int i = 0; i < joints.size(); i++)
        desc.joints[joints[i]] = settings[i];
}

std::vector<JointData> AR60xHWDriver::JointGetSettings(std::vector<uint8_t> joints)
{
    RECV_GUARD;
    std::vector<JointData> data(joints.size());
    for(int i = 0; i<joints.size(); i++)
        data[i] = desc.joints[joints[i]];
}


// position
void AR60xHWDriver::JointSetPosition(uint8_t joint, double position)
{
    SEND_GUARD;
    sendpacket_->jointSetPosition(joint, position);
}

double AR60xHWDriver::JointGetPosition(uint8_t joint)
{
    RECV_GUARD;
    return recv_packet_->jointGetPosition(joint);
}

void AR60xHWDriver::JointSetPosition(std::vector<uint8_t>& joints, std::vector<double>& position)
{
    SEND_GUARD;
    if(joints.size() != position.size())
        throw std::invalid_argument("Count of joints and position must be equal");

    for(int i = 0; i < joints.size(); i++)
        sendpacket_->jointSetPosition(joints[i], position[i]);
}

std::vector<double> AR60xHWDriver::JointGetPosition(std::vector<uint8_t>& joints)
{
    RECV_GUARD;
    std::vector<double> positions(joints.size());
    for(int i = 0; i < joints.size(); i++)
        positions[i] = recv_packet_->jointGetPosition(joints[i]);

    return positions;
}


// offset
void AR60xHWDriver::JointSetOffset(uint8_t joint, double offset)
{
    SEND_GUARD;
    desc.joints[joint].offset = offset;
    sendpacket_->jointSetOffset(joint, offset);
}


double AR60xHWDriver::JointGetOffset(uint8_t joint)
{
    RECV_GUARD;
    return desc.joints[joint].offset;
}

void AR60xHWDriver::JointSetOffset(std::vector<uint8_t> joints, std::vector<double> offset)
{
    SEND_GUARD;
    if(joints.size() != offset.size())
        throw std::invalid_argument("Count of joints and offset must be equal");

    for(int i = 0; i < joints.size(); i++)
    {
        sendpacket_->jointSetOffset(joints[i], offset[i]);
        desc.joints[joints[i]].offset = offset[i];
    }
}

std::vector<double> AR60xHWDriver::JointGetOffset(std::vector<uint8_t> joints)
{
    RECV_GUARD;
    std::vector<double> offsets(joints.size());
    for(int i = 0; i < joints.size(); i++)
        offsets[i] = desc.joints[joints[i]].offset;

    return offsets;
}


// reverse
void AR60xHWDriver::JointSetReverce(uint8_t joint, bool isReverce)
{
    SEND_GUARD;
    desc.joints[joint].isReverse = isReverce;
}

bool AR60xHWDriver::JointGetReverce(uint8_t joint)
{
    RECV_GUARD;
    return desc.joints[joint].isReverse;
}

void AR60xHWDriver::JointSetReverce(std::vector<uint8_t> joints, std::vector<bool> isReverse)
{
    SEND_GUARD;
    if(joints.size() != isReverse.size())
        throw std::invalid_argument("Count of joints and isReverse must be equal");

    for(int i = 0; i < joints.size(); i++)
        desc.joints[joints[i]].isReverse = isReverse[i];

}

std::vector<bool> AR60xHWDriver::JointGetReverce(std::vector<uint8_t> joints)
{
    RECV_GUARD;
    std::vector<bool> isReverse(joints.size());
    for(int i = 0; i<joints.size(); i++)
        isReverse[i] = desc.joints[joints[i]].isReverse;

    return isReverse;
}


// PID
void AR60xHWDriver::JointSetPIDGains(uint8_t joint, JointData::PIDGains gains)
{
    SEND_GUARD;
    //TODO: Write to desc
    sendpacket_->jointSetPIDGains(joint, gains);
}

JointData::PIDGains AR60xHWDriver::JointGetPIDGains(uint8_t joint)
{
    RECV_GUARD;
    //TODO: Read real pids from robot?
    return desc.joints[joint].gains;
}

void AR60xHWDriver::JointSetPIDGains(std::vector<uint8_t> joints, std::vector<JointData::PIDGains> gains)
{
    SEND_GUARD;
    if (joints.size() != gains.size())
        throw std::invalid_argument("Count of joints and gains must be equal");

    for (int i = 0; i < joints.size(); i++)
    {
        //TODO: Write to desc
        sendpacket_->jointSetPIDGains(joints[i], gains[i]);
    }
}

std::vector<JointData::PIDGains> AR60xHWDriver::JointGetPIDGains(std::vector<uint8_t> joints)
{
    RECV_GUARD;
    std::vector<JointData::PIDGains> pids(joints.size());
    for(int i = 0; i<joints.size(); i++)
        pids[i] = desc.joints[joints[i]].gains;

    return pids;
}


// limits
void AR60xHWDriver::JointSetLimits(uint8_t joint, JointData::JointLimits limits)
{
    SEND_GUARD;
    desc.joints[joint].limits = limits;
    sendpacket_->jointSetLowerLimit(joint, limits.lowerLimit);
    sendpacket_->jointSetUpperLimit(joint, limits.upperLimit);
}

// TODO: Удалить JointSettings перенсти вместо него JointInformation!!!!!
JointData::JointLimits AR60xHWDriver::JointGetLimits(uint8_t joint)
{
    RECV_GUARD;
    return desc.joints[joint].limits;
}

void AR60xHWDriver::JointSetLimits(std::vector<uint8_t> joints, std::vector<JointData::JointLimits> limits)
{
    SEND_GUARD;
    if(joints.size() != limits.size())
        throw std::invalid_argument("Count of joints and limits must be equal");

    for(int i = 0; i<joints.size(); i++)
    {
        desc.joints[joints[i]].limits = limits[i];
        sendpacket_->jointSetLowerLimit(joints[i], limits[i].lowerLimit);
        sendpacket_->jointSetUpperLimit(joints[i], limits[i].upperLimit);
    }
}

std::vector<JointData::JointLimits> AR60xHWDriver::JointGetLimits(std::vector<uint8_t> joints)
{
    RECV_GUARD;
    std::vector<JointData::JointLimits> limits(joints.size());
    for(int i = 0; i<joints.size(); i++)
        limits[i] = desc.joints[joints[i]].limits;

    return limits;
}


// enable
void AR60xHWDriver::JointSetEnable(uint8_t joint, bool isEnable)
{
    SEND_GUARD;
    desc.joints[joint].isEnable = isEnable;
}

bool AR60xHWDriver::JointGetEnable(uint8_t joint)
{
    RECV_GUARD;
    return desc.joints[joint].isEnable;
}

void AR60xHWDriver::JointSetEnable(std::vector<uint8_t> joints, std::vector<bool> isEnable)
{
    SEND_GUARD;
    if(joints.size() != isEnable.size())
        throw std::invalid_argument("Count of joints and isEnable must be equal");

    for(int i = 0; i<joints.size(); i++)
        desc.joints[joints[i]].isEnable = isEnable[i];
}

std::vector<bool> AR60xHWDriver::JointGetEnable(std::vector<uint8_t> joints)
{
    RECV_GUARD;
    std::vector<bool> isEnable(joints.size());
    for(int i = 0; i < joints.size(); i++)
        isEnable[i] = desc.joints[joints[i]].isEnable;

    return isEnable;
}

// state
void AR60xHWDriver::JointSetState(uint8_t joint, JointState::MotorState motorState, JointState::ControlType controlType)
{
    SEND_GUARD;
    JointState state;
    state.state = motorState;
    state.controlType = controlType;
    sendpacket_->jointSetState(joint, state);
}

//TODO: проверить!!!!
JointState AR60xHWDriver::JointGetState(uint8_t joint)
{
    RECV_GUARD;
    return recv_packet_->jointGetState(joint);
}

void AR60xHWDriver::JointSetState(std::vector<uint8_t> joints, std::vector<JointState> state)
{
    SEND_GUARD;
    if(joints.size() != state.size())
        throw std::invalid_argument("Count of joints and state must be equal");

    for(int i = 0; i<joints.size(); i++)
        sendpacket_->jointSetState(joints[i], state[i]);
}

std::vector<JointState> AR60xHWDriver::JointGetState(std::vector<uint8_t> joints)
{
    RECV_GUARD;
    std::vector<JointState> states(joints.size());
    for(int  i = 0; i<joints.size(); i++)
        states[i] = recv_packet_->jointGetState(joints[i]);

    return states;
}

// ------------------------------ power control -----------------------------------------

PowerState::PowerSupplyState AR60xHWDriver::JointGetSupplyState(uint8_t joint)
{
    RECV_GUARD;
    return recv_packet_->jointGetSupplyState(joint);
}



std::vector<PowerState::PowerSupplyState> AR60xHWDriver::JointsGetSupplyState(std::vector<uint8_t> joints)
{
    RECV_GUARD;
    std::vector<PowerState::PowerSupplyState> state(joints.size());
    for(int i = 0; i<joints.size(); i++)
        state[i] = recv_packet_->jointGetSupplyState(joints[i]);

    return state;
}

PowerState::PowerSupplyState AR60xHWDriver::PowerGetSupplyState(PowerData::PowerSupplies supply)
{
    RECV_GUARD;
    return recv_packet_->supplyGetState(supply);
}

void AR60xHWDriver::SupplySetOnOff(PowerData::PowerSupplies supply, bool onOffState)
{
    SEND_GUARD;
    if(onOffState)
        sendpacket_->supplySetOn(supply);
    else
        sendpacket_->supplySetOff(supply);
}

bool AR60xHWDriver::SupplyGetOnOff(PowerData::PowerSupplies supply)
{
    //RECV_GUARD;
    //TODO: нет метода в recvpacket
    throw std::runtime_error("Not implemented");
}

// ------------------------------ sensorGroups -----------------------------------------------

double AR60xHWDriver::SensorGetState(int sensor)
{
    RECV_GUARD;
    return recv_packet_->sensorGetValue(sensor);
}

SensorImuState AR60xHWDriver::SensorGetImu()
{
    RECV_GUARD;
    return recv_packet_->sensorGetImu();
}

SensorFeetState AR60xHWDriver::SensorGetFeet()
{
    RECV_GUARD;
    return recv_packet_->sensorGetFeet();
}






sensor_msgs::JointState AR60xHWDriver::JointsGetState()
{
    RECV_GUARD;
    return recv_packet_->JointsGetState();
}

void AR60xHWDriver::JointsSetCommand(robot_controller_ros::JointsCommand command)
{
    SEND_GUARD;


}


robot_controller_ros::JointsParams AR60xHWDriver::JointsGetParams()
{
    return robot_controller_ros::JointsParams();
}

void AR60xHWDriver::JointsSetMode(robot_controller_ros::JointsMode mode)
{

}

robot_controller_ros::SourcesSupplyState AR60xHWDriver::PowerGetSourcesSupplyState()
{
    return robot_controller_ros::SourcesSupplyState();
}

robot_controller_ros::JointsSupplyState AR60xHWDriver::PowerGetJointsSupplyState()
{
    return robot_controller_ros::JointsSupplyState();
}


