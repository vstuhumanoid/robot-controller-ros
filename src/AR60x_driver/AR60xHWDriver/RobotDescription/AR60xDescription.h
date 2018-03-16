#ifndef AR60XDESCRIPTION_H
#define AR60XDESCRIPTION_H

#include <map>

#include "../DataTypes/JointData.h"
#include "../DataTypes/SensorData.h"


/**
 * \brief Robot configuration (joints, sensors)
 */
class AR60xDescription
{
public:

    AR60xDescription();

    std::map<int, SensorData> sensors;
    std::map<int, JointData> joints;
};

#endif // AR60XDESCRIPTION_H
