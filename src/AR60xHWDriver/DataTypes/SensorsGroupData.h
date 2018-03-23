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

    /**
     * Channel - address of sensor sub-packet in
     * all packet
     */
    uint8_t channel;

    /**
     * List of sensors in group
     */
    std::vector<SensorData> sensors;
};

#endif //ROBOT_CONTROLLER_ROS_SENSORSGROUP_H
