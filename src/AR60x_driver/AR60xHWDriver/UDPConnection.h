#ifndef UDPCONNECTION_H
#define UDPCONNECTION_H

#include <QObject>
#include <QThread>
#include <QUdpSocket>
#include <QTime>
#include <QDebug>
#include <QTimer>

#include <mutex>
#include <stdlib.h>

#include "RobotPackets/AR60xRecvPacket.h"
#include "RobotPackets/AR60xSendPacket.h"
#include <thread>
#include <chrono>

class UDPConnection
{

public:
    UDPConnection();
    ~UDPConnection(){}

    void connectToHost(std::string host, int sendPort, int delay);
    void breakConnection();
    void initPackets();

    void setRecvPacket(AR60xRecvPacket * packet) { recvPacket = packet; }
    void setSendPacket(AR60xSendPacket * packet) { sendPacket = packet; }


private:
    QUdpSocket *socket;

    std::mutex *sendLocker;
    std::mutex *recvLocker;

    QString host;
    int sendPort;
    int sendDelay;

    volatile bool isRunning;

    AR60xRecvPacket *recvPacket;
    AR60xSendPacket *sendPacket;

    std::thread update_thread;

    void printConnectionState();

    void thread_func();
    void receiveDatagram();
    void sendDatagram();

};

#endif // UDPCONNECTION_H
