#ifndef AR60XHWDRIVER_H
#define AR60XHWDRIVER_H

#include "RobotDescription/AR60xDescription.h"
#include "RobotPackets/AR60xRecvPacket.h"
#include "RobotPackets/AR60xSendPacket.h"
#include "HardwareInterfaces/IAR60xJointControl.h"
#include "HardwareInterfaces/IAR60xPowerControl.h"
#include "HardwareInterfaces/IAR60xSensorState.h"
#include "XMLSerializer.h"
#include "UDPConnection.h"

#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <exception>


class AR60xHWDriver :
        IAR60xJointControl,
        IAR60xPowerControl,
        IAR60xSensorState
{
public:

    /**
     * Create new AR60x driver
     * @param max_recv_packet_size Maximum size of receive buffer (bytes)
     */
    AR60xHWDriver(size_t max_recv_packet_size);

    /**
     * Create new AR60x driver
     * @param config_filename Path to the robot's config file
     * @param max_recv_packet_size Maximum size of receive buffer (bytes)
     */
    AR60xHWDriver(std::string config_filename, size_t max_recv_packet_size);
    ~AR60xHWDriver();

    /**
     * Load robot's config file
     * @param fileName Path to the robot's config file
     */
    void loadConfig(std::string fileName);
    bool saveConfig(std::string fileName);

    // interfaces
    void robotConnect();
    void robotDisconnect();

    // ------------------------------- Joint control interface ---------------------------------------------------------

    void JointSetSettings(int joint, JointData settings) override;
    JointData JointGetSettings(int joint) override;
    void JointSetSettings(std::vector<int> joints, std::vector<JointData> settings) override ;
    std::vector<JointData> JointGetSettings(std::vector<int> joints) override ;

    void JointSetPosition(int joint, int position) override;
    int JointGetPosition(int joint) override;
    void JointSetPosition(std::vector<int>& joints, std::vector<int>& position) override ;
    std::vector<int> JointGetPosition(std::vector<int>& joints) override ;

    void JointSetOffset(int joint, int offset) override;
    int JointGetOffset(int joint) override;
    void JointSetOffset(std::vector<int> joints, std::vector<int> offset) override ;
    std::vector<int> JointGetOffset(std::vector<int> joints) override ;

    void JointSetReverce(int joint, bool isReverce) override;
    bool JointGetReverce(int joint) override;
    void JointSetReverce(std::vector<int> joints, std::vector<bool> isReverse) override ;
    std::vector<bool> JointGetReverce(std::vector<int> joints) override ;

    void JointSetPIDGains(int joint, JointData::PIDGains gains) override;
    JointData::PIDGains JointGetPIDGains(int joint) override;
    void JointSetPIDGains(std::vector<int> joints, std::vector<JointData::PIDGains> gains) override ;
    std::vector<JointData::PIDGains> JointGetPIDGains(std::vector<int> joints) override ;

    void JointSetLimits(int joint, JointData::JointLimits limits) override;
    JointData::JointLimits JointGetLimits(int joint) override;
    void JointSetLimits(std::vector<int> joints, std::vector<JointData::JointLimits> limits) override ;
    std::vector<JointData::JointLimits> JointGetLimits(std::vector<int> joints) override ;

    void JointSetEnable(int joint, bool isEnable) override;
    bool JointGetEnable(int joint) override;
    void JointSetEnable(std::vector<int> joints, std::vector<bool> isEnable) override ;
    std::vector<bool> JointGetEnable(std::vector<int> joints) override ;

    void JointSetState(int joint, JointState::JointStates state) override;
    JointState JointGetState(int joint) override;
    void JointSetState(std::vector<int> joints, std::vector<JointState::JointStates> state) override ;
    std::vector<JointState> JointGetState(std::vector<int> joints) override ;


    // ------------------------------- Power control interface ---------------------------------------------------------

    /**
     * Get voltage and current of specific joint
     * @param joint Joint's number
     * @return Joint's parameters
     */
    PowerState::PowerSupplyState JointGetSupplyState(int joint) override;

    /**
     * Get voltage and current of specific supply source (48v, 12v etc)
     * @param supply Selected supply
     * @return Supply's parameters
     */
    PowerState::PowerSupplyState PowerGetSupplyState(PowerData::PowerSupplies supply) override;

    /**
     * Set supply source on/off
     * @param supply Selected supply
     * @param onOffState On or off
     */
    void SupplySetOnOff(PowerData::PowerSupplies supply, bool onOffState) override;

    /**
     * Get supply state (on/off)
     * @param supply Selected supply
     * @return on or off
     */
    bool SupplyGetOnOff(PowerData::PowerSupplies supply) override;


    // -------------------------------------- Sensors -------- ---------------------------------------------------------

    SensorState SensorGetState(int sensor) override;

    AR60xDescription *getRobotDesc();


private:
    void init_packets();

    AR60xDescription desc;

    UDPConnection *connection;
    AR60xRecvPacket *recvPacket;
    AR60xSendPacket *sendpacket;

    ConnectionData connectionData;
    size_t max_recv_packet_size_;

};

#endif // AR60XHWDRIVER_H
