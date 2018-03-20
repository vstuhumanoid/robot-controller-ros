#ifndef JOINTDATA_H
#define JOINTDATA_H

#include <string>

class JointData
{
public:

    /**
     * PID-controller gains
     */
    struct PIDGains
    {
        uint16_t proportional;
        uint16_t integral;
        uint16_t derivative;
    };

    /**
     * Joint's angle limits
     */
    struct JointLimits
    {
        double lowerLimit;
        double upperLimit;
    };

    JointData();

    uint8_t number;         // Joint number
    uint8_t channel;        // Joint channel in package (shift in package)
    std::string name;       // Joint name
    PIDGains gains;         // PID-controller gains
    JointLimits limits;     // Angle limits
    double offset;          // Initial position
    bool isReverse;         // Shoulde reverse angles
    bool isEnable;          // Enable the joint
};

#endif // JOINTDATA_H
