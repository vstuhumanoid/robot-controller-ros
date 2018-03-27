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


AR60xDescription& AR60xHWDriver::GetRobotDesc()
{
    return desc_;
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


////////////////////////////////// JOINTS CONTROL //////////////////////////////////////////////////////////////////////

sensor_msgs::JointState AR60xHWDriver::JointsGetState()
{
    RECV_GUARD;
    return recv_packet_->JointsGetState();
}

void AR60xHWDriver::JointsSetPosition(robot_controller_ros::JointsCommand command)
{
    SEND_GUARD;

    if((command.names.size() != command.positions.size()) && (command.pids.size() == 0 || command.pids.size() != command.names.size()))
    {
        ROS_ERROR_STREAM("JointsSetCommand: names and positions vectors should be same size. pids should be zero or same size");
        ROS_ERROR_STREAM("Ignoring command");
        return;
    }

    for(int i = 0; i < command.names.size(); i++)
    {
        JointData& joint = desc_.joints[atoi(command.names[i].c_str())];
        sendpacket_->jointSetPosition(joint, command.positions[i]);

        if(command.pids.size() > 0)
            sendpacket_->jointSetPIDGains(joint, command.pids[i]);
    }
}



void AR60xHWDriver::JointsGetParams()
{
}

void AR60xHWDriver::JointsSetParams(robot_controller_ros::JointsParams params)
{
    SEND_GUARD;

    if(params.names.size() != params.params.size())
    {
        ROS_ERROR_STREAM("JointSetParams: names and params vectors should be same size");
        ROS_ERROR_STREAM("Ignoring command");
        return;
    }

    for(int i = 0; i < params.names.size(); i++)
    {
        JointData& joint = desc_.joints[atoi(params.names[i].c_str())];

        sendpacket_->jointSetReverse(joint, params.params[i].reverse);
        sendpacket_->jointSetLimits(joint, params.params[i].lower_limit, params.params[i].upper_limit);
        sendpacket_->jointSetOffset(joint, params.params[i].offset);
        sendpacket_->jointSetMode(joint, params.params[i].mode);
        //TODO: PIDs
        //TODO: Enable
    }
}



void AR60xHWDriver::JointsSetMode(robot_controller_ros::JointsMode mode)
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
        sendpacket_->jointSetMode(joint, mode.modes[i]);
    }
}

//////////////////////////////////// POWER SUPPLY CONTROL //////////////////////////////////////////////////////////////

robot_controller_ros::SourcesSupplyState AR60xHWDriver::PowerGetSourcesSupplyState()
{
    return robot_controller_ros::SourcesSupplyState();
}

robot_controller_ros::JointsSupplyState AR60xHWDriver::PowerGetJointsSupplyState()
{
    return robot_controller_ros::JointsSupplyState();
}


void AR60xHWDriver::SupplySetOnOff(PowerData::PowerSupplies supply, bool onOffState)
{
    SEND_GUARD;

    sendpacket_->supplySetOnOff(supply, onOffState);
}

