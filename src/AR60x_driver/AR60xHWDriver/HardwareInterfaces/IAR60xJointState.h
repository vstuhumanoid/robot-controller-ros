#pragma once

#include "../DataTypes/JointData.h"
#include "../DataTypes/JointState.h"
#include "../DataTypes/PowerState.h"

class IAR60xJointState
{
public:
    virtual JointData JointGetSettings(int joint) = 0;

    virtual int JointGetPosition(int joint) = 0;
	virtual JointState JointGetState(int joint) = 0;
    virtual bool JointGetReverce(int joint) = 0;
    virtual PowerState::PowerSupplyState JointGetSupplyState(int joint) = 0;
    virtual JointData::JointLimits JointGetLimits(int joint) = 0;
    virtual bool JointGetEnable(int joint) = 0;
    virtual JointData::PIDGains JointGetPIDGains(int joint) = 0;
};

