//
// Created by user on 22.03.18.
//

#ifndef ROBOT_CONTROLLER_ROS_SENSORSGROUP_H
#define ROBOT_CONTROLLER_ROS_SENSORSGROUP_H

#include <string>
#include <vector>
#include "SensorData.h"

/**
 * Group of same sensors config
 */
struct SensorsGroup
{
    int id;
    std::string name;
    int channel;
    std::vector<SensorData> sensors;
};

#endif //ROBOT_CONTROLLER_ROS_SENSORSGROUP_H
