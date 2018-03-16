#pragma once

#include "../DataTypes/SensorState.h"

class IAR60xSensorState
{
public:
    virtual SensorState SensorGetState(int sensor) = 0;
};

