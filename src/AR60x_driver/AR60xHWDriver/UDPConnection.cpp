#include "UDPConnection.h"

UDPConnection::UDPConnection(QObject *parent) : QThread(parent)
{
    isRunning = false;
    time = new QTime;
}

void UDPConnection::run()
{
    socket = new QUdpSocket();

    connect(socket, SIGNAL(readyRead()), SLOT(processPendingDatagrams()),Qt::DirectConnection);

    qDebug() << "UDPConnection - binding..." << endl;
    if (!socket->bind(recvPort, QUdpSocket::ShareAddress))
        qDebug()<< "UDPConnection - Not Bind!";

    sendTimer = new QTimer;
    sendTimer->setInterval(sendDelay);
    connect(sendTimer, SIGNAL(timeout()), SLOT(sendDatagram()), Qt::DirectConnection);
    sendTimer->start(sendDelay);

    printConnectionState();

    exec();

    qDebug() << "UDPConnection - disconnect..." << endl;
    disconnect(sendTimer, SIGNAL(timeout()));
    disconnect(socket, SIGNAL(readyRead()));

    socket->disconnect();
    socket->close();
    printConnectionState();

}

void UDPConnection::connectToHost(std::string host, int sendPort, int recvPort, int delay)
{
    this->host = QString::fromStdString( host );
    this->sendPort = sendPort;
    this->recvPort = recvPort;
    this->sendDelay = delay;

    if(isRunning == false)
    {
        isRunning = true;
        start();
    }
}

void UDPConnection::breakConnection()
{
    if(isRunning == true)
    {
        isRunning = false;
        exit();
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

void UDPConnection::sendDatagram()
{
    QHostAddress address = QHostAddress(host);
    sendLocker->lock();
    socket->writeDatagram(sendPacket->getByteArray(), sendPacket->getSize() * sizeof(char), address, sendPort);
    socket->waitForBytesWritten();
    sendLocker->unlock();

    qDebug() << "UDPConnection - sended at " << QTime::currentTime().toString("hh:mm:ss.zzz") << endl;
}

void UDPConnection::processPendingDatagrams()
{
    while (socket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(socket->pendingDatagramSize());
        QHostAddress Host;
        quint16 Port;

        socket->readDatagram(datagram.data(), datagram.size(), &Host, &Port);
        recvPacket->initFromByteArray( datagram.data() );

        emit dataReady();
     }
}
