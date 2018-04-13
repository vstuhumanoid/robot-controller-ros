#ifndef JOINTDATA_H
#define JOINTDATA_H

#include <string>
#include <robot_controller_ros/TypePid.h>

using namespace robot_controller_ros;

/**
 * Joint settings from config
 */
struct JointData
{
    std::string name;           ///< Joint's name. Used as identifier and should be equals with URDF joint names
    std::string description;    ///< Joint's description
    uint8_t number;             ///< Joint's number (also is used as identifier)
    uint8_t channel;            ///< Joint's channel in package (shift in package)
    TypePid pid_gains;          ///< PID-controller gains
    double lower_limit;         ///< Angle lower limit (in degrees)
    double upper_limit;         ///< Angle upper limit (in degrees)
    double offset;              ///< Joint's zero position (in degrees)
    bool is_reverse;            ///< Shoulde reverse angles
    bool is_enable;             ///< Enable the joint
};

#endif // JOINTDATA_H
