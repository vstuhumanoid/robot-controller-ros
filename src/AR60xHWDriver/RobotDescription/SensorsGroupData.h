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
struct SensorsGroupData
{
    int id;
    std::string name;
    uint8_t channel;                   ///< Address of sensor sub-packet in all packet
    std::vector<SensorData> sensors;   ///< List of sensors in group
};

#endif //ROBOT_CONTROLLER_ROS_SENSORSGROUP_H
