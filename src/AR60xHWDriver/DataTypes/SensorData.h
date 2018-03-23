#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <string>

/**
 * Sensor config
 */
struct SensorData
{
    /**
     * Sensor number - special sensor identifier in
     * package
     */
    uint8_t number;
    double offset;
    std::string name;
};

#endif // SENSORDATA_H
