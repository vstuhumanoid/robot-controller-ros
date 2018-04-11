#ifndef CONNECTIONDATA_H
#define CONNECTIONDATA_H

#include <string>

/**
 * Settings for robot network connection
 */
struct ConnectionData
{
    std::string host;  ///< Robot's IP address
    int robotPort;     ///< Robot's port
    int localPort;     ///< Local port, because ephemerial ports isn't supported by AR600
    int sendDelay;     ///< Sending loop delay in ms
};
#endif // CONNECTIONDATA_H
