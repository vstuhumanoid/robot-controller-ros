#pragma once

#include "../DataTypes/JointData.h"
#include "../DataTypes/JointState.h"

class IAR60xJointControl
{
public:

    virtual void JointSetSettings(int joint, JointData settings) = 0;
    virtual JointData JointGetSettings(int joint) = 0;
    virtual void JointSetSettings(std::vector<int> joints, std::vector<JointData> settings) = 0;
    virtual std::vector<JointData> JointGetSettings(std::vector<int> joints) = 0;

    virtual void JointSetPosition(int joint, int position) = 0;
    virtual int JointGetPosition(int joint) = 0;
    virtual void JointSetPosition(std::vector<int>& joints, std::vector<int>& position) = 0;
    virtual std::vector<int> JointGetPosition(std::vector<int>& joints) = 0;

    virtual void JointSetOffset(int joint, int offset) = 0;
    virtual int JointGetOffset(int joint) = 0;
    virtual void JointSetOffset(std::vector<int> joints, std::vector<int> offset) = 0;
    virtual std::vector<int> JointGetOffset(std::vector<int> joints) = 0;

    virtual void JointSetReverce(int joint, bool isReverce) = 0;
    virtual bool JointGetReverce(int joint) = 0;
    virtual void JointSetReverce(std::vector<int> joints, std::vector<bool> isReverse) = 0;
    virtual std::vector<bool> JointGetReverce(std::vector<int> joints) = 0;

    virtual void JointSetPIDGains(int joint, JointData::PIDGains gains) = 0;
    virtual JointData::PIDGains JointGetPIDGains(int joint) = 0;
    virtual void JointSetPIDGains(std::vector<int> joints, std::vector<JointData::PIDGains> gains) = 0;
    virtual std::vector<JointData::PIDGains> JointGetPIDGains(std::vector<int> joints) = 0;

    virtual void JointSetLimits(int joint, JointData::JointLimits limits) = 0;
    virtual JointData::JointLimits JointGetLimits(int joint) = 0;
    virtual void JointSetLimits(std::vector<int> joints, std::vector<JointData::JointLimits> limits) = 0;
    virtual std::vector<JointData::JointLimits> JointGetLimits(std::vector<int> joints) = 0;

    virtual void JointSetEnable(int joint, bool isEnable) = 0;
    virtual bool JointGetEnable(int joint) = 0;
    virtual void JointSetEnable(std::vector<int> joints, std::vector<bool> isEnable) = 0;
    virtual std::vector<bool> JointGetEnable(std::vector<int> joints) = 0;

    virtual void JointSetState(int joint, JointState::JointStates state) = 0;
    virtual JointState JointGetState(int joint) = 0;
    virtual void JointSetState(std::vector<int> joints, std::vector<JointState::JointStates> state) = 0;
    virtual std::vector<JointState> JointGetState(std::vector<int> joints) = 0;


};

