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

class UDPConnection : public QThread
{
    Q_OBJECT
public:
    explicit UDPConnection(QObject *parent = 0);
    ~UDPConnection(){}

    void run();
    void connectToHost(std::string host, int sendPort, int recvPort, int delay);
    void breakConnection();
    void initPackets();

    void setRecvPacket(AR60xRecvPacket * packet) { recvPacket = packet; }
    void setSendPacket(AR60xSendPacket * packet) { sendPacket = packet; }

signals:
    void dataReady();
private:
    QUdpSocket *socket;
    QTimer *sendTimer;
    QTime *time;

    std::mutex *sendLocker;
    std::mutex *recvLocker;

    QString host;
    int sendPort;
    int sendDelay;
    int recvPort;

    volatile bool isRunning;

    AR60xRecvPacket *recvPacket;
    AR60xSendPacket *sendPacket;

    void printConnectionState();

private slots:
    void sendDatagram();
    void processPendingDatagrams();
};

#endif // UDPCONNECTION_H
