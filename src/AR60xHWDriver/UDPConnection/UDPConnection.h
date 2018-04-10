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
     * @param recvLocker Mutex for receiving
     * @param sendLocker Mutex for sending
     * @param localPort port socket is binding to
     * @param delay Sending loop delay (ms)
     */
    UDPConnection(AR60xSendPacket& sendPacket,
                  AR60xRecvPacket& recvPacket,
                  std::mutex& sendLocker,
                  std::mutex& recvLocker,
                  uint16_t localPort,
                  uint32_t delay);
    ~UDPConnection();

    /**
     * Connect to remote host and start communication thread
     * @param robot_address Robot's IP-address
     * @param robot_port Robots' port
     */
    void ConnectToHost(std::string robot_address, uint16_t local_port, uint16_t robot_port);

    /**
     * Disconnect from remote host and stop communication thread
     */
    void BreakConnection();

    void Send();
    void Receive();
private:

    // send & recv packages sync
    std::mutex& send_locker_;
    std::mutex& recv_locker_;

    // send loop delay
    int send_delay_;

    // connection
    io_service io_service_;
    ip::udp::socket socket_;

    // packages
    AR60xRecvPacket& recv_packet_;
    AR60xSendPacket& send_packet_;

    bool is_connected_;

};

#endif // UDPCONNECTION_H
