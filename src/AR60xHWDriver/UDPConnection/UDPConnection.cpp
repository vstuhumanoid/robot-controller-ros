#include <asio/ip/udp.hpp>
#include "UDPConnection.h"

using namespace std;

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
    send_locker_(sendLocker),
    fucking_asio_thread(&UDPConnection::fucking_asio_thread_func, this)
{
    fucking_asio_thread.detach();
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
        //TODO: Check error code
        size_t sended = socket_.send(buffer(send_packet_.getByteArray(), send_packet_.getSize() * sizeof(char)));
        if(sended != packetSize)
        {
            ROS_ERROR("Sending error");
            ROS_ERROR("Unknown error");
            connection_failed();
        }
        else
            connection_established();
    }
    catch(const std::runtime_error& er)
    {
        ROS_ERROR("Sending error");
        ROS_ERROR_STREAM(er.what());
        connection_failed();
    }


    send_locker_.unlock();
}

void UDPConnection::Receive()
{
    recv_locker_.lock();

    try
    {
        auto future = receive(socket_, recv_packet_.getByteArray(), packetSize);
        auto status = future.wait_for(2s);
        if(status == future_status::timeout)
        {
            socket_.cancel();
            ROS_WARN("Receiving timeout");
            connection_failed();
        }
        else
        {
            SocketResult sr = future.get();
            if(sr.ErrorCode)
            {
                ROS_WARN_STREAM("Receiving error: " << sr.ErrorCode.message());
                connection_failed();
            }
            else
            {
                connection_established();
                //ROS_INFO("OK");
            }
        }

    }
    catch(const std::runtime_error& er)
    {
        ROS_ERROR("Receiving error");
        ROS_ERROR_STREAM(er.what());
        connection_failed();
    }

    recv_locker_.unlock();
}

std::future<UDPConnection::SocketResult>  UDPConnection::receive(ip::udp::socket& socket, uint8_t *buf, size_t buffer_size)
{
    auto promise = std::make_shared<std::promise<SocketResult>>();
    socket.async_receive(buffer(buf, buffer_size), [promise](error_code code, size_t received)
    {
        promise->set_value({code, received});
    });

    return promise->get_future();
}

std::future<UDPConnection::SocketResult>  UDPConnection::send(ip::udp::socket& socket, uint8_t *buf, size_t buffer_size)
{
    auto promise = std::make_shared<std::promise<SocketResult>>();
    socket.async_send(buffer(buf, buffer_size), [promise](error_code code, size_t received)
    {
        promise->set_value({code, received});
    });

    return promise->get_future();
}

void UDPConnection::fucking_asio_thread_func()
{
    io_service::work work(io_service_);
    io_service_.run();
}
