#ifndef CONNECTIONDATA_H
#define CONNECTIONDATA_H

#include <string>

class ConnectionData
{
public:
    ConnectionData();

    std::string host;
    int sendPort;
    int recvPort;
    int sendDelay;
};

#endif // CONNECTIONDATA_H
