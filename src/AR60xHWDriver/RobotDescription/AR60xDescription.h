#ifndef AR60XDESCRIPTION_H
#define AR60XDESCRIPTION_H

#include <map>
#include "JointData.h"
#include "SensorsGroupData.h"


/**
 * \brief Robot configuration (joints, sensors)
 */
class AR60xDescription
{
public:
    AR60xDescription(){

    }
    AR60xDescription(const AR60xDescription&) = delete;

    std::map<int, SensorsGroupData> sensorGroups;
    std::map<int, JointData> joints;
};

#endif // AR60XDESCRIPTION_H
