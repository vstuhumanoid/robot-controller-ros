#ifndef UDPCONNECTION_H
#define UDPCONNECTION_H

#include <string>
#include <thread>
#include <chrono>
#include <mutex>

#include "RobotPackets/AR60xRecvPacket.h"
#include "RobotPackets/AR60xSendPacket.h"


#include <asio.hpp>
#include <ros/ros.h>
#include <cstdint>

using namespace asio;

class UDPConnection
{

public:
    UDPConnection(size_t max_recv_buffer_size);
    ~UDPConnection();

    void connectToHost(std::string host, unsigned short sendPort, int delay);
    void breakConnection();
    void initPackets();

    void setRecvPacket(AR60xRecvPacket * packet) { recv_packet_ = packet; }
    void setSendPacket(AR60xSendPacket * packet) { send_packet_ = packet; }


private:

    // send & recv packages sync
    std::mutex *sendLocker;
    std::mutex *recvLocker;

    // send loop delay
    int send_delay_;

    // connection
    ip::udp::endpoint robot_endpoint_;
    io_service io_service_;
    ip::udp::socket socket_;


    volatile bool is_running_;

    // packages
    AR60xRecvPacket *recv_packet_;
    AR60xSendPacket *send_packet_;
    uint8_t* recv_buffer_;
    size_t max_recv_buffer_size_;

    // sending & receiving thread
    std::thread update_thread;

    void thread_func();
    void send_datagram();
    void receive_datagram();

};

#endif // UDPCONNECTION_H
