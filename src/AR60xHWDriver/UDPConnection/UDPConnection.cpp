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
    this->send_delay_ = delay;

    // bind socket to local ephemerial port
    // TODO: Maybe robot expects fixed port
    try
    {
        socket_.open(ip::udp::v4());
        socket_.bind(ip::udp::endpoint(ip::udp::v4(), localPort));
    }
    catch(const std::runtime_error& er)
    {
        ROS_ERROR("Unable to bind socket");
        ROS_ERROR_STREAM(er.what());
    }
}

UDPConnection::~UDPConnection()
{
    breakConnection();
}


void UDPConnection::connectToHost(std::string robot_address, uint16_t robot_port)
{
    if (!is_running_)
    {
        // Connect to robot.
        // Connection is used just to use send instead send_to later.
        // It's bit faster
        // TODO: Check endpoint is correct
        // TODO: Return value or exception when error
        try
        {
            auto robot_endpoint_ = ip::udp::endpoint(ip::address::from_string(robot_address), robot_port);
            socket_.connect(robot_endpoint_);
        }
        catch(const std::runtime_error& er)
        {
            ROS_ERROR("Unable to connect to robot");
            ROS_ERROR_STREAM(er.what());
            return;
        }

        ROS_INFO_STREAM("Connected to " << robot_address << ":" <<  robot_port);

        update_thread = std::thread(&UDPConnection::thread_func, this);
        is_running_ = true;
        update_thread.detach();

        ROS_INFO("Connection thread started with perioud %dms", this->send_delay_);
    }
    else
    {
        ROS_WARN("Already connected");
    }
}

void UDPConnection::breakConnection()
{
    if(is_running_)
    {
        ROS_INFO("Stopping...");
        is_running_ = false;
        update_thread.join();
        socket_.close();
        ROS_INFO("Stopped & disconnected");
    }
}


void UDPConnection::thread_func()
{
    while (is_running_)
    {
        send_datagram();
        receive_datagram();
        std::this_thread::sleep_for(std::chrono::milliseconds(send_delay_));
    }
}

void UDPConnection::send_datagram()
{
    send_locker_.lock();

    try
    {
        //TODO: Handle errors and disconnection
        size_t sended = socket_.send(buffer(send_packet_.getByteArray(), send_packet_.getSize() * sizeof(char)));
    }
    catch(const std::runtime_error& er)
    {
        ROS_ERROR("Sending error");
        ROS_ERROR_STREAM(er.what());
    }


    send_locker_.unlock();
}

void UDPConnection::receive_datagram()
{
    recv_locker_.lock();

    try
    {
        //TODO: Handle errors and disconnection
        size_t received = socket_.receive(buffer(recv_packet_.getByteArray(), packetSize));
    }
    catch(const std::runtime_error& er)
    {
        ROS_ERROR("Receiving error");
        ROS_ERROR_STREAM(er.what());
    }

    recv_locker_.unlock();
}
