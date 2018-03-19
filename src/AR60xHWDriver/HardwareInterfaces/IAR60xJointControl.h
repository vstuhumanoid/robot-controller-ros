#pragma once

#include "../DataTypes/JointData.h"
#include "../DataTypes/JointState.h"

class IAR60xJointControl
{
public:

    virtual void JointSetSettings(int joint, JointData settings) = 0;

    virtual void JointSetPosition(int joint, int position) = 0;
    virtual void JointSetOffset(int joint, int offset) = 0;
    virtual void JointSetReverce(int joint, bool isReverce) = 0;
    virtual void JointSetPIDGains(int joint, JointData::PIDGains gains) = 0;
    virtual void JointSetLimits(int joint, JointData::JointLimits limits) = 0;
    virtual void JointSetEnable(int joint, bool isEnable) = 0;
    virtual void JointSetState(int joint, JointState::JointStates state) = 0;
};

