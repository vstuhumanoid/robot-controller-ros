#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <string>

/**
 * Sensor's settings from config
 */
struct SensorData
{
    uint8_t number;         ///< special sensor identifier in package
    double offset;          ///< Zero value
    std::string name;       ///< Display name
};

#endif // SENSORDATA_H
