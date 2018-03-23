#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <string>

class SensorData
{
public:
    SensorData();

    int number;            // Номер датчика
    std::string name;      // Имя датчика
};

#endif // SENSORDATA_H
