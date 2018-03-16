#ifndef JOINTDATA_H
#define JOINTDATA_H

#include <string>

class JointData
{
public:
    JointData();

    int number;        // Номер узла
    std::string name;  // Имя узла

    int channel;            // Номер канала в пакете управления

    struct PIDGains         // Структура регулятора
    {
        int proportional;
        int integral;
        int derivative;
    };

    PIDGains gains;         // Коэффициенты регулирования

    struct JointLimits      // Структура пределов
    {
        int lowerLimit;
        int upperLimit;
    };

    JointLimits limits;     // Пределы

    int offset;             // Смещение абсолютного положения
    bool isReverce;         // Реверсировать углы
    bool isEnable;          // Задействовать узел
};

#endif // JOINTDATA_H
