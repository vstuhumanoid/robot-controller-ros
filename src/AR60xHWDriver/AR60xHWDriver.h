#ifndef AR60XHWDRIVER_H
#define AR60XHWDRIVER_H

#include "RobotDescription/AR60xDescription.h"
#include "RobotPackets/AR60xRecvPacket.h"
#include "RobotPackets/AR60xSendPacket.h"
#include "XMLSerializer/XMLSerializer.h"
#include "UDPConnection/UDPConnection.h"

#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <exception>
#include <mutex>
#include <robot_controller_ros/FeetSensors.h>
#include <robot_controller_ros/JointsCommand.h>
#include <robot_controller_ros/JointsMode.h>
#include <robot_controller_ros/JointsParams.h>
#include <robot_controller_ros/JointsSupplyState.h>
#include <robot_controller_ros/SourcesSupplyState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <ros/time.h>

/**
 * @brief Main AR60x robot driver class
 *
 * Provide methods to control robot's joints, power supply and
 * get sensors' data
 *
 * Functions:
 *  - Joints control: set position, set limits and offset (zero position)
 *  - Power supply control: on/off power sources, get voltage and current for
 *       every power source and every joint's motor
 *  - Get sensors data
 *  - Get sensors data
 */
class AR60xHWDriver
{
public:
    AR60xDescription desc_;
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
    void LoadConfig(std::string fileName);

    /**
     * Save robot's config to file
     * @param fileName Path to the config file
     * @return Success
     */
    bool SaveConfig(std::string fileName);

    /**
     *  Connect to the robot
     */
    void RobotConnect();

    /**
     * Disconnect from the robot
     */
    void RobotDisconnect();


    /**
     * Set TRACE mode for all joints and set their
     * position to 0 deg
     */
    void SetStartPose();


    // ------------------------------- Joint control interface ---------------------------------------------------------

    /**
     * @brief Get all joints current state
     *
     * Get position, velocity and effort not supported
     * @return
     */
    sensor_msgs::JointState JointsGetState();

    /**
      * Send position command to joints
      *
      * This is a main method to control joints. Set joints' position
      * and PID-controller gains
      *
      * @param command ROS message with joints names, positions and PID-gains
      */
    void JointsSetCommand(const robot_controller_ros::JointsCommand command);

    /**
     * Get joints params (names, enable, limits, offset, reverse, mode, pid gains);
     */
    robot_controller_ros::JointsParams JointsGetParams();

    /**
     * @brief Setup joints params (enable, limits, offset, reverse, mode, pid gains)
     *
     * All vectors should be same size or empty. If you don't want to set some parameters
     * (want to keep current value) just pass empty vector
     *
    * @param params ROS message with joints params
    */
    void JointsSetParams(const robot_controller_ros::JointsParams params);

    /**
     * Control joint's motor and clutch
     * @param mode ROS message with  joint's operational mode
     */
    void JointsSetMode(const robot_controller_ros::JointsMode mode);



    // ------------------------------- Power control interface ---------------------------------------------------------

    /**
     * Get supply state (voltage, current) of all power sources
     * @return
     */
    robot_controller_ros::SourcesSupplyState PowerGetSourcesSupplyState();

    /**
     * Get all joints supply state (voltage, current) of all joints
     * @return
     */
    robot_controller_ros::JointsSupplyState PowerGetJointsSupplyState();

    /**
     * Set supply source on/off
     * @param supply Selected supply
     * @param onOffState On or off
     */
    void SupplySetOnOff(const PowerSources supply, const bool onOffState);


    // -------------------------------------- Sensors -------- ---------------------------------------------------------

    /**
     * Get data from IMU sensor
     * @return
     */
    sensor_msgs::Imu SensorGetImu();

    /**
     * Set calibration value (zero offset) for IMU sensor
     * @param imu calibration value
     */
    void SensorSetImuCalibration(const sensor_msgs::Imu imu);


    /**
     * Get data from feet sensors
     * @return
     */
    robot_controller_ros::FeetSensors SensorGetFeet();

    /**
     * Set calibration value (zero offset) for feet pressure sensors
     * @param feet calibration values
     */
    void SensorSetFeetCalibration(const SensorFeetState feet);


private:

#define RECV_GUARD std::lock_guard<std::mutex> guard(recv_mutex_)
#define SEND_GUARD std::lock_guard<std::mutex> guard(send_mutex_)

    void init_packets();
    bool check_sizes(const JointsParams &params) const;
    bool equal_or_empty(int size, int orig_size) const;
    void set_timestamp(std_msgs::Header& header);

    JointData* find_joint(std::string name);

    ConnectionData connectionData;
    UDPConnection *connection_;
    AR60xRecvPacket *recv_packet_;
    AR60xSendPacket *sendpacket_;

    std::mutex send_mutex_, recv_mutex_;
};

#endif // AR60XHWDRIVER_H
