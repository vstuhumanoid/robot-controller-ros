#pragma once

class IAR60xSensorState
{
public:
    virtual double SensorGetState(int sensor) = 0;
    virtual SensorImuState SensorGetImu() = 0;
    virtual SensorFeetState SensorGetFeet() = 0;
};

