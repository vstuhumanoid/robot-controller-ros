#ifndef AR60XHWDRIVER_H
#define AR60XHWDRIVER_H

#include "RobotDescription/AR60xDescription.h"

#include "RobotPackets/AR60xRecvPacket.h"
#include "RobotPackets/AR60xSendPacket.h"

#include "HardwareInterfaces/IAR60xJointControl.h"
#include "HardwareInterfaces/IAR60xJointState.h"
#include "HardwareInterfaces/IAR60xPowerControl.h"
#include "HardwareInterfaces/IAR60xPowerState.h"
#include "HardwareInterfaces/IAR60xSensorState.h"

#include "XMLSerializer.h"

#include "UDPConnection.h"

#include <string>
#include <fstream>
#include <sstream>

#include <QFile>
#include <QJsonArray>

class AR60xHWDriver :
        IAR60xJointControl,
        IAR60xJointState,
        IAR60xPowerControl,
        IAR60xPowerState,
        IAR60xSensorState
{
private:
    AR60xDescription * desc;

    UDPConnection *connection;
    AR60xRecvPacket *recvPacket;
    AR60xSendPacket *sendpacket;

    XMLSerializer * XMLser;

    ConnectionData connectionData;
public:
    AR60xHWDriver();
    bool saveConfig(std::string fileName);
    bool loadConfig(std::string fileName);
    void initPackets();

    // interfaces
    void robotConnect();
    void robotDisconnect();

    // Joint control interface
    void JointSetSettings(int joint, JointData settings) override;
    JointData JointGetSettings(int joint) override;

    void JointSetPosition(int joint, int position) override;
    int JointGetPosition(int joint) override;

    void JointSetOffset(int joint, int offset) override;

    void JointSetReverce(int joint, bool isReverce) override;
    bool JointGetReverce(int joint) override;

    void JointSetPIDGains(int joint, JointData::PIDGains gains) override;
    JointData::PIDGains JointGetPIDGains(int joint) override;

    void JointSetLimits(int joint, JointData::JointLimits limits) override;
    JointData::JointLimits JointGetLimits(int joint) override;

    void JointSetEnable(int joint, bool isEnable) override;
    bool JointGetEnable(int joint) override;

    void JointSetState(int joint, JointState::JointStates state) override;
    JointState JointGetState(int joint) override;


    // Power control interface
    PowerState::PowerSupplyState JointGetSupplyState(int joint) override;
    PowerState::PowerSupplyState PowerGetSupplyState(PowerData::PowerSupplies supply) override;
    void PowerSetSettings(PowerData settings) override;
    void SupplySetState(PowerData::PowerSupplies supply, bool onOffState) override;
    bool PowerGetOnOff(PowerData::PowerSupplies supply) override;

    SensorState SensorGetState(int sensor) override;

    AR60xDescription *getRobotDesc();

};

#endif // AR60XHWDRIVER_H
