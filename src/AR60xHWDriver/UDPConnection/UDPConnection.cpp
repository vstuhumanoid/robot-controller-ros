#include <asio/ip/udp.hpp>
#include "UDPConnection.h"

UDPConnection::UDPConnection(AR60xSendPacket& sendPacket,
                             AR60xRecvPacket& recvPacket,
                             std::mutex& sendLocker,
                             std::mutex& recvLocker,
                             uint16_t localPort,
                             uint32_t delay)
    :io_service_(),
    socket_(io_service_),
    send_packet_(sendPacket),
    recv_packet_(recvPacket),
    recv_locker_(recvLocker),
    send_locker_(sendLocker)
{
    send_delay_ = delay;
    is_connected_ = false;


}

UDPConnection::~UDPConnection()
{
    BreakConnection();

    if(socket_.is_open())
        socket_.close();
}


void UDPConnection::ConnectToHost(std::string robot_address, uint16_t  local_port, uint16_t robot_port)
{
    if (!is_connected_)
    {
        // bind socket to fixed local port
        // ar600 doesn't support ephemerial port
        try
        {
            ROS_INFO_STREAM("Binding socket to local port " << local_port << "...");
            socket_.open(ip::udp::v4());
            socket_.bind(ip::udp::endpoint(ip::udp::v4(), local_port));
        }
        catch(const std::runtime_error& er)
        {
            ROS_ERROR("Unable to bind socket");
            ROS_ERROR_STREAM(er.what());

            throw std::runtime_error("Unable to bind socket");
        }

        ROS_INFO("Done");

        // Connect to robot.
        // Connection is used just to use send instead send_to later.
        // It's bit faster
        // TODO: Check endpoint is correct
        // TODO: Return value or exception when error
        try
        {
            ROS_INFO_STREAM("Connecting socket to robot " << robot_address <<":" << robot_port << "...");
            auto robot_endpoint_ = ip::udp::endpoint(ip::address::from_string(robot_address), robot_port);
            socket_.connect(robot_endpoint_);
        }
        catch(const std::runtime_error& er)
        {
            ROS_ERROR("Unable to connect to robot");
            ROS_ERROR_STREAM(er.what());

            throw std::runtime_error("Unable to connect to robot");
        }

        ROS_INFO("Done");
        is_connected_ = true;
    }
    else
    {
        ROS_WARN("Already connected");
    }
}

void UDPConnection::BreakConnection()
{
    if(is_connected_)
    {
        ROS_INFO("Disconnecting...");
        is_connected_ = false;
        socket_.close();
        ROS_INFO("Disconnected");
    }
    else
    {
        ROS_WARN("Not connected");
    }
}



void UDPConnection::Send()
{
    send_locker_.lock();

    try
    {
        size_t sended = socket_.send(buffer(send_packet_.getByteArray(), send_packet_.getSize() * sizeof(char)));
        if(sended != packetSize)
        {
            ROS_ERROR("Sending error");
            ROS_ERROR("Unknown error");
        }
    }
    catch(const std::runtime_error& er)
    {
        ROS_ERROR("Sending error");
        ROS_ERROR_STREAM(er.what());
    }


    send_locker_.unlock();
}

void UDPConnection::Receive()
{
    recv_locker_.lock();

    try
    {
        //TODO: timeout
        size_t received = socket_.receive(buffer(recv_packet_.getByteArray(), packetSize));
        if (received != packetSize)
        {
            ROS_ERROR("Receiving error");
            ROS_ERROR("Unknown error");
        }
    }
    catch(const std::runtime_error& er)
    {
        ROS_ERROR("Receiving error");
        ROS_ERROR_STREAM(er.what());
    }

    recv_locker_.unlock();
}
