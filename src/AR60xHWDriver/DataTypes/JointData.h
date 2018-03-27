#ifndef JOINTDATA_H
#define JOINTDATA_H

#include <string>
#include <robot_controller_ros/TypePid.h>

using namespace robot_controller_ros;

struct JointData
{
    uint8_t number;         // Joint number
    uint8_t channel;        // Joint channel in package (shift in package)
    std::string name;       // Joint name
    TypePid gains;          // PID-controller gains
    double lower_limit;     // Angle limits
    double upper_limit;
    double offset;          // Initial position
    bool isReverse;         // Shoulde reverse angles
    bool isEnable;          // Enable the joint
};

#endif // JOINTDATA_H
