#pragma once

#include <DataTypes/JointData.h>
#include <DataTypes/JointState.h>
#include <vector>

class IAR60xJointControl
{
public:

    virtual void JointSetSettings(uint8_t joint, JointData settings) = 0;
    virtual JointData JointGetSettings(uint8_t joint) = 0;
    virtual void JointSetSettings(std::vector<uint8_t> joints, std::vector<JointData> settings) = 0;
    virtual std::vector<JointData> JointGetSettings(std::vector<uint8_t> joints) = 0;

    virtual void JointSetPosition(uint8_t joint, double position) = 0;
    virtual double JointGetPosition(uint8_t joint) = 0;
    virtual void JointSetPosition(std::vector<uint8_t>& joints, std::vector<double>& position) = 0;
    virtual std::vector<double> JointGetPosition(std::vector<uint8_t>& joints) = 0;

    virtual void JointSetOffset(uint8_t joint, double offset) = 0;
    virtual double JointGetOffset(uint8_t joint) = 0;
    virtual void JointSetOffset(std::vector<uint8_t> joints, std::vector<double> offset) = 0;
    virtual std::vector<double> JointGetOffset(std::vector<uint8_t> joints) = 0;

    virtual void JointSetReverce(uint8_t joint, bool isReverce) = 0;
    virtual bool JointGetReverce(uint8_t joint) = 0;
    virtual void JointSetReverce(std::vector<uint8_t> joints, std::vector<bool> isReverse) = 0;
    virtual std::vector<bool> JointGetReverce(std::vector<uint8_t> joints) = 0;

    virtual void JointSetPIDGains(uint8_t joint, JointData::PIDGains gains) = 0;
    virtual JointData::PIDGains JointGetPIDGains(uint8_t joint) = 0;
    virtual void JointSetPIDGains(std::vector<uint8_t> joints, std::vector<JointData::PIDGains> gains) = 0;
    virtual std::vector<JointData::PIDGains> JointGetPIDGains(std::vector<uint8_t> joints) = 0;

    virtual void JointSetLimits(uint8_t joint, JointData::JointLimits limits) = 0;
    virtual JointData::JointLimits JointGetLimits(uint8_t joint) = 0;
    virtual void JointSetLimits(std::vector<uint8_t> joints, std::vector<JointData::JointLimits> limits) = 0;
    virtual std::vector<JointData::JointLimits> JointGetLimits(std::vector<uint8_t> joints) = 0;

    virtual void JointSetEnable(uint8_t joint, bool isEnable) = 0;
    virtual bool JointGetEnable(uint8_t joint) = 0;
    virtual void JointSetEnable(std::vector<uint8_t> joints, std::vector<bool> isEnable) = 0;
    virtual std::vector<bool> JointGetEnable(std::vector<uint8_t> joints) = 0;

    virtual void JointSetState(uint8_t joint, JointState::JointStates state) = 0;
    virtual JointState JointGetState(uint8_t joint) = 0;
    virtual void JointSetState(std::vector<uint8_t> joints, std::vector<JointState::JointStates> state) = 0;
    virtual std::vector<JointState> JointGetState(std::vector<uint8_t> joints) = 0;
};

