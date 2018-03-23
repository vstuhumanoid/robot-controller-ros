#ifndef CONNECTIONDATA_H
#define CONNECTIONDATA_H

#include <string>

struct ConnectionData
{
    std::string host;
    int localPort;
    int robotPort;
    int sendDelay;
};

#endif // CONNECTIONDATA_H
