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
    if(!serializer.deserialize(fileName, &desc_, &connectionData))
    {
        ROS_ERROR("Config parsing error");
        throw std::runtime_error("Config parsing error");
    }

    init_packets();
}

bool AR60xHWDriver::SaveConfig(std::string fileName)
{
    XMLSerializer serializer;
    return serializer.serialize(fileName, &desc_, &connectionData);
}

void AR60xHWDriver::init_packets()
{
    sendpacket_ = new AR60xSendPacket(desc_);
    recv_packet_ = new AR60xRecvPacket(desc_);
    recv_packet_->initFromByteArray(sendpacket_->getByteArray());
    connection_ = new UDPConnection(*sendpacket_, *recv_packet_,
                                    recv_mutex_, send_mutex_,
                                    connectionData.localPort,
                                    connectionData.sendDelay);
}


////////////////////////////////////// CONNECTION //////////////////////////////////////////////////////////////////////


void AR60xHWDriver::RobotConnect()
{
    connection_->connectToHost(connectionData.host, connectionData.robotPort);
}

void AR60xHWDriver::RobotDisconnect()
{
    connection_->breakConnection();
}


////////////////////////////////// JOINTS CONTROL //////////////////////////////////////////////////////////////////////

sensor_msgs::JointState AR60xHWDriver::JointsGetState()
{
    RECV_GUARD;

    sensor_msgs::JointState msg;
    msg.name.resize(desc_.joints.size());
    msg.position.resize(desc_.joints.size());

    int i = 0;
    for(auto& joint: desc_.joints)
    {
        msg.name[i] = std::to_string(joint.second.number);
        msg.position[i] = recv_packet_->JointGetPosition(joint.second);
        i++;
    }

    return msg;
}

void AR60xHWDriver::JointsSetPosition(const robot_controller_ros::JointsCommand command)
{
    if((command.names.size() != command.positions.size()) || (command.pids.size() != 0 && command.pids.size() != command.names.size()))
    {
        ROS_ERROR_STREAM("JointsSetCommand: names and positions vectors should be same size. pids should be zero or same size");
        ROS_ERROR_STREAM("Ignoring command");
        return;
    }

    SEND_GUARD;

    for(int i = 0; i < command.names.size(); i++)
    {
        JointData& joint = desc_.joints[atoi(command.names[i].c_str())];
        sendpacket_->JointSetPosition(joint, command.positions[i]);

        if(command.pids.size() > 0)
            sendpacket_->JointSetPIDGains(joint, command.pids[i]);
    }
}



robot_controller_ros::JointsParams AR60xHWDriver::JointsGetParams()
{
    robot_controller_ros::JointsParams msg;
    msg.names.resize(desc_.joints.size());
    msg.lower_limit.resize(desc_.joints.size());
    msg.upper_limit.resize(desc_.joints.size());
    msg.offset.resize(desc_.joints.size());
    msg.reverse.resize(desc_.joints.size());
    msg.enabled.resize(desc_.joints.size());
    msg.mode.resize(desc_.joints.size());
    msg.pids.resize(desc_.joints.size());

    int i = 0;
    for(auto& joint: desc_.joints)
    {
        msg.names[i] = std::to_string(joint.second.number);
        msg.lower_limit[i]  = recv_packet_->JointGetLowerLimit(joint.second);
        msg.upper_limit[i]  = recv_packet_->JointGetUpperLimit(joint.second);
        msg.offset[i] = recv_packet_->JointGetOffset(joint.second);
        msg.reverse[i] = joint.second.is_reverse;
        msg.enabled[i] = joint.second.is_enable;
        msg.mode[i] = recv_packet_->JointGetMode(joint.second);
        msg.pids[i] = recv_packet_->JointGetPidGains(joint.second);
        i++;
    }

    return msg;
}

void AR60xHWDriver::JointsSetParams(const robot_controller_ros::JointsParams params)
{
    if(!check_sizes(params))
        return;

    SEND_GUARD;

    for(int i = 0; i < params.names.size(); i++)
    {
        JointData& joint = desc_.joints[atoi(params.names[i].c_str())];

        if(params.reverse.size() != 0)
            sendpacket_->JointSetReverse(joint, params.reverse[i]);

        if(params.lower_limit.size() != 0 && params.upper_limit.size() != 0)
            sendpacket_->JointSetLimits(joint, params.lower_limit[i], params.upper_limit[i]);

        if(params.offset.size() != 0)
            sendpacket_->JointSetOffset(joint, params.offset[i]);

        if(params.mode.size() != 0)
            sendpacket_->JointSetMode(joint, params.mode[i]);

        if(params.pids.size() != 0)
            sendpacket_->JointSetPIDGains(joint, params.pids[i]);

        joint.is_enable = params.enabled[i];
    }
}


void AR60xHWDriver::JointsSetMode(const robot_controller_ros::JointsMode mode)
{
    SEND_GUARD;

    if(mode.names.size() != mode.modes.size())
    {
        ROS_ERROR_STREAM("JointSetParams: names and modes vectors should be same size");
        ROS_ERROR_STREAM("Ignoring command");
        return;
    }

    for(int i = 0; i<mode.names.size(); i++)
    {
        JointData& joint = desc_.joints[atoi(mode.names[i].c_str())];
        sendpacket_->JointSetMode(joint, mode.modes[i]);
    }
}

//////////////////////////////////// POWER SUPPLY CONTROL //////////////////////////////////////////////////////////////

robot_controller_ros::SourcesSupplyState AR60xHWDriver::PowerGetSourcesSupplyState()
{
    RECV_GUARD;

    robot_controller_ros::SourcesSupplyState msg;
    msg.S48 = recv_packet_->PowerGetSourceSupplyState(PowerSources::Supply48V);
    msg.S12 = recv_packet_->PowerGetSourceSupplyState(PowerSources::Supply12V);
    msg.S8_1 = recv_packet_->PowerGetSourceSupplyState(PowerSources::Supply8V1);
    msg.S8_2 = recv_packet_->PowerGetSourceSupplyState(PowerSources::Supply8V2);
    msg.S6_1 = recv_packet_->PowerGetSourceSupplyState(PowerSources::Supply6V1);
    msg.S6_2 = recv_packet_->PowerGetSourceSupplyState(PowerSources::Supply6V2);
    return msg;
}

robot_controller_ros::JointsSupplyState AR60xHWDriver::PowerGetJointsSupplyState()
{
    RECV_GUARD;

    robot_controller_ros::JointsSupplyState msg;
    msg.names.resize(desc_.joints.size());
    msg.states.resize(desc_.joints.size());

    int i = 0;
    for(auto& joint: desc_.joints)
    {
        msg.names[i] = std::to_string(joint.second.number);
        msg.states[i] = recv_packet_->PowerGetJointSupplyState(joint.second);
        i++;
    }

    return msg;
}


void AR60xHWDriver::SupplySetOnOff(const PowerSources supply, const bool onOffState)
{
    SEND_GUARD;
    sendpacket_->PowerSourceSetOnOff(supply, onOffState);
}

////////////////////////////////////// SENSORS /////////////////////////////////////////////////////////////////////////

sensor_msgs::Imu AR60xHWDriver::SensorGetImu()
{
    RECV_GUARD;
    return recv_packet_->SensorsGetImu();
}


void AR60xHWDriver::SensorSetImuCalibration(const sensor_msgs::Imu imu)
{
    SEND_GUARD;
    sendpacket_->SensorSetImuCalibration(imu);
}

SensorFeetState AR60xHWDriver::SensorGetFeet()
{
    RECV_GUARD;
    return recv_packet_->SensorsGetFeet();
}

void AR60xHWDriver::SensorSetFeetCalibration(const SensorFeetState feet)
{
    SEND_GUARD;
    sendpacket_->SensorSetFeetCalibration(feet);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool AR60xHWDriver::check_sizes(const JointsParams &params) const
{
    bool is_pid_size_ok = equal_or_empty(params.pids.size(), params.names.size());
    bool is_upper_size_ok = equal_or_empty(params.upper_limit.size(), params.names.size());
    bool is_lower_size_ok = equal_or_empty(params.lower_limit.size(), params.names.size());
    bool is_offset_size_ok = equal_or_empty(params.offset.size(), params.names.size());
    bool is_reverse_size_ok = equal_or_empty(params.reverse.size(), params.names.size());
    bool is_enabled_size_ok = equal_or_empty(params.enabled.size(), params.names.size());
    bool is_mode_size_ok =equal_or_empty(params.mode.size(), params.names.size());

    if(!is_pid_size_ok || !is_upper_size_ok || !is_lower_size_ok || !is_offset_size_ok || !is_reverse_size_ok || !is_enabled_size_ok || !is_mode_size_ok)
    {
        ROS_ERROR_STREAM("JointSetParams: all vectors should be same size or empty");
        ROS_ERROR_STREAM("Ignoring command");
        return false;
    }

    return true;
}

bool AR60xHWDriver::equal_or_empty(int size, int orig_size) const
{
    return size == 0 || size == orig_size;
}


