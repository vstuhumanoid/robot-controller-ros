#include "AR60xDescription.h"

AR60xDescription::AR60xDescription()
{

}

std::map<int, SensorData> *AR60xDescription::getSensors()
{
    return &sensors;
}

std::map<int, JointData> *AR60xDescription::getJoints()
{
    return &joints;
}
