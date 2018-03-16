#include "UDPConnection.h"

UDPConnection::UDPConnection()
{
    socket = NULL;

}


void UDPConnection::connectToHost(std::string host,  int sendPort, int delay)
{
    if(isRunning == false)
    {

        if(socket != NULL)
            delete socket;


        this->host = QString::fromStdString( host );
        this->sendPort = sendPort;
        this->sendDelay = delay;

        socket = new QUdpSocket();
        qDebug() << "UDPConnection - binding..." << endl;
        if (!socket->bind(10001, QUdpSocket::ShareAddress))
            qDebug()<< "UDPConnection - Not Bind!";

        printConnectionState();

        update_thread = std::thread(&UDPConnection::thread_func, this);
        isRunning = true;
        update_thread.detach();
    }
}

void UDPConnection::breakConnection()
{
    if(isRunning == true)
    {
        isRunning = false;

        update_thread.join();

        socket->disconnect();
        socket->close();

        printConnectionState();
    }
}

void UDPConnection::initPackets()
{
    recvLocker = recvPacket->getMutex();
    sendLocker = sendPacket->getMutex();
    sendPacket->init();
    recvPacket->initFromByteArray(sendPacket->getByteArray());
}

void UDPConnection::printConnectionState()
{
    if (socket->state() == QUdpSocket::BoundState)
    {
        qDebug() << "UDPConnection - bounded";
    }
    else
    {
        qDebug() << "UDPConnection - unbounded";
    }
}


void UDPConnection::thread_func()
{
    while (isRunning)
    {
        sendDatagram();
        receiveDatagram();
        std::this_thread::sleep_for(std::chrono::milliseconds(sendDelay));
    }
}

void UDPConnection::sendDatagram()
{
    QHostAddress address = QHostAddress(host);
    sendLocker->lock();
    socket->writeDatagram(sendPacket->getByteArray(), sendPacket->getSize() * sizeof(char), address, sendPort);
    socket->waitForBytesWritten();
    sendLocker->unlock();

    //qDebug() << "UDPConnection - sended at " << QTime::currentTime().toString("hh:mm:ss.zzz") << endl;
}

void UDPConnection::receiveDatagram()
{
    QByteArray datagram;
    datagram.resize(socket->pendingDatagramSize());
    QHostAddress Host;
    quint16 Port;

    socket->readDatagram(datagram.data(), datagram.size(), &Host, &Port);
    recvPacket->initFromByteArray( datagram.data() );
}
