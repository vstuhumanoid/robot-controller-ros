#ifndef AR60XHWDRIVER_H
#define AR60XHWDRIVER_H

#include "RobotDescription/AR60xDescription.h"
#include "RobotPackets/AR60xRecvPacket.h"
#include "RobotPackets/AR60xSendPacket.h"
#include "HardwareInterfaces/IAR60xJointControl.h"
#include "HardwareInterfaces/IAR60xPowerControl.h"
#include "HardwareInterfaces/IAR60xSensorState.h"
#include "XMLSerializer/XMLSerializer.h"
#include "UDPConnection/UDPConnection.h"

#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <exception>
#include <mutex>


class AR60xHWDriver :
        IAR60xJointControl,
        IAR60xPowerControl,
        IAR60xSensorState
{
public:

    /**
     * Create new AR60x driver
     */
    AR60xHWDriver();

    /**
     * Create new AR60x driver
     * @param config_filename Path to the robot's config file
     */
    AR60xHWDriver(std::string config_filename);
    ~AR60xHWDriver();

    /**
     * Load robot's config file
     * @param fileName Path to the robot's config file
     */
    void loadConfig(std::string fileName);

    /**
     * Save robot's config to file
     * @param fileName Path to the config file
     * @return Success
     */
    bool saveConfig(std::string fileName);

    // interfaces
    void robotConnect();
    void robotDisconnect();

    // ------------------------------- Joint control interface ---------------------------------------------------------

    void SetStartPose();

    void JointSetSettings(uint8_t joint, JointData settings) override;
    JointData JointGetSettings(uint8_t joint) override;
    void JointSetSettings(std::vector<uint8_t> joints, std::vector<JointData> settings) override ;
    std::vector<JointData> JointGetSettings(std::vector<uint8_t> joints) override ;

    void JointSetPosition(uint8_t joint, double position) override;
    double JointGetPosition(uint8_t joint) override;
    void JointSetPosition(std::vector<uint8_t>& joints, std::vector<double>& position) override ;
    std::vector<double> JointGetPosition(std::vector<uint8_t>& joints) override ;

    void JointSetOffset(uint8_t joint, double offset) override;
    double JointGetOffset(uint8_t joint) override;
    void JointSetOffset(std::vector<uint8_t> joints, std::vector<double> offset) override ;
    std::vector<double> JointGetOffset(std::vector<uint8_t> joints) override ;

    void JointSetReverce(uint8_t joint, bool isReverce) override;
    bool JointGetReverce(uint8_t joint) override;
    void JointSetReverce(std::vector<uint8_t> joints, std::vector<bool> isReverse) override ;
    std::vector<bool> JointGetReverce(std::vector<uint8_t> joints) override ;

    void JointSetPIDGains(uint8_t joint, JointData::PIDGains gains) override;
    JointData::PIDGains JointGetPIDGains(uint8_t joint) override;
    void JointSetPIDGains(std::vector<uint8_t> joints, std::vector<JointData::PIDGains> gains) override ;
    std::vector<JointData::PIDGains> JointGetPIDGains(std::vector<uint8_t> joints) override ;

    void JointSetLimits(uint8_t joint, JointData::JointLimits limits) override;
    JointData::JointLimits JointGetLimits(uint8_t joint) override;
    void JointSetLimits(std::vector<uint8_t> joints, std::vector<JointData::JointLimits> limits) override ;
    std::vector<JointData::JointLimits> JointGetLimits(std::vector<uint8_t> joints) override ;

    void JointSetEnable(uint8_t joint, bool isEnable) override;
    bool JointGetEnable(uint8_t joint) override;
    void JointSetEnable(std::vector<uint8_t> joints, std::vector<bool> isEnable) override ;
    std::vector<bool> JointGetEnable(std::vector<uint8_t> joints) override ;

    void JointSetState(uint8_t joint,
                       JointState::MotorState motorState,
                       JointState::ControlType controlType = JointState::ControlType::POSITION_CONTROl) override;
    JointState JointGetState(uint8_t joint) override;
    void JointSetState(std::vector<uint8_t> joints, std::vector<JointState> state) override ;
    std::vector<JointState> JointGetState(std::vector<uint8_t> joints) override ;


    // ------------------------------- Power control interface ---------------------------------------------------------

    /**
     * Get voltage and current of specific joint
     * @param joint Joint's number
     * @return Joint's parameters
     */
    PowerState::PowerSupplyState JointGetSupplyState(uint8_t joint) override;

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

    double SensorGetState(int sensor) override;
    SensorImuState SensorGetImu() override ;
    SensorFeetState SensorGetFeet() override ;

    AR60xDescription *getRobotDesc();


private:

#define RECV_GUARD std::lock_guard<std::mutex> guard(recv_mutex_)
#define SEND_GUARD std::lock_guard<std::mutex> guard(send_mutex_)

    void init_packets();

    AR60xDescription desc;

    UDPConnection *connection_;
    AR60xRecvPacket *recv_packet_;
    AR60xSendPacket *sendpacket_;

    ConnectionData connectionData;
    size_t max_recv_packet_size_;

    std::mutex send_mutex_, recv_mutex_;

};

#endif // AR60XHWDRIVER_H
