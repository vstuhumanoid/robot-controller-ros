#include <asio/ip/udp.hpp>
#include "UDPConnection.h"

using namespace std;

UDPConnection::UDPConnection(shared_ptr<AR60xSendPacket> sendPacket,
                             shared_ptr<AR60xRecvPacket> recvPacket,
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

        robot_endpoint_ = ip::udp::endpoint(ip::address::from_string(robot_address), robot_port);
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



bool UDPConnection::Send()
{
    std::lock_guard<std::mutex> guard(send_locker_);

    try
    {
        error_code code;
        size_t sended = socket_.send_to(buffer(send_packet_->getByteArray(), packetSize), robot_endpoint_, 0, code);
        if(code)
        {
            ROS_WARN_STREAM("Sending error: " << code.message());
            ROS_ERROR("Unknown error");
            connection_failed();
            return false;
        }

        connection_established();
        return true;
    }
    catch(const std::runtime_error& er)
    {
        ROS_ERROR("Sending error");
        ROS_ERROR_STREAM(er.what());
        connection_failed();
        return false;
    }
}

bool UDPConnection::Receive()
{
    std::lock_guard<std::mutex> guard(recv_locker_);

    try
    {
        auto future = receive(socket_, recv_packet_->getByteArray(), packetSize);
        auto status = future.wait_for(2s);
        if(status == future_status::timeout)
        {
            socket_.cancel();
            ROS_WARN("Receiving timeout");
            connection_failed();
            return false;
        }
        else
        {
            SocketResult sr = future.get();
            if(sr.ErrorCode)
            {
                ROS_WARN_STREAM("Receiving error: " << sr.ErrorCode.message());
                connection_failed();
                return false;
            }
            else
            {
                connection_established();
                return true;
            }
        }

    }
    catch(const std::runtime_error& er)
    {
        ROS_ERROR("Receiving error");
        ROS_ERROR_STREAM(er.what());
        connection_failed();
        return false;
    }
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
