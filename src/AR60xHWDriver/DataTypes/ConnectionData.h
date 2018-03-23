#ifndef CONNECTIONDATA_H
#define CONNECTIONDATA_H

#include <string>

struct ConnectionData
{
    std::string host;
    int sendPort;
    int recvPort;
    int sendDelay;
};

#endif // CONNECTIONDATA_H
