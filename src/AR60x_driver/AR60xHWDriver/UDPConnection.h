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

    /**
     * Create new connection object
     * @param sendPacket Reference to the send packet structure
     * @param recvPacket Referenct to the recv packet structure
     * @param delay Sending loop delay (ms)
     * @param max_recv_buffer_size  maximum size of receive buffer (bytes)
     */
    UDPConnection(AR60xSendPacket& sendPacket,
                  AR60xRecvPacket& recvPacket,
                  uint32_t delay,
                  size_t max_recv_buffer_size);
    ~UDPConnection();

    /**
     * Connect to remote host and start communication thread
     * @param robot_address Robot's IP-address
     * @param robot_port Robots' port
     */
    void connectToHost(std::string robot_address, uint16_t robot_port);

    /**
     * Disconnect from remote host and stop communication thread
     */
    void breakConnection();


private:

    // send & recv packages sync
    std::mutex *sendLocker;
    std::mutex *recvLocker;

    // send loop delay
    int send_delay_;

    // connection
    io_service io_service_;
    ip::udp::socket socket_;


    volatile bool is_running_;

    // packages
    AR60xRecvPacket& recv_packet_;
    AR60xSendPacket& send_packet_;
    uint8_t* recv_buffer_;
    size_t max_recv_buffer_size_;

    // sending & receiving thread
    std::thread update_thread;

    void thread_func();
    void send_datagram();
    void receive_datagram();

};

#endif // UDPCONNECTION_H
