#ifndef SENSORSTATE_H
#define SENSORSTATE_H

#include <string>

/*
* Состояния датчиков робота
*/
class SensorState
{
public:

	float sensorValue; // Значение величины
	std::string name; // Имя датчика

	SensorState();
	~SensorState();
};

#endif // SENSORSTATE_H
