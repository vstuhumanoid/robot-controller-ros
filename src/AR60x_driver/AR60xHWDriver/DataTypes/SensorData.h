#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <string>

class SensorData
{
public:
    SensorData();

    int number;       // Номер датчика
    int channel;            // Канал в пакете управления
    std::string name; // Имя датчика

    enum SensorGroups       // Группы датчиков
    {
        IMUSensors,
        FootPressureSensors
    };

    SensorGroups group;     // Группа датчика
};

#endif // SENSORDATA_H
